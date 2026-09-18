# Car Battery Charger — ESP32-C3-Zero smart lead-acid charger

Standalone smart charger for flooded/AGM 12V car batteries (44-100Ah).
No WiFi/MQTT/cloud/OTA — everything is local: an INA219 current/voltage
sensor, a SH1106 128x64 OLED, and a rotary encoder with push button are
the entire interface. Re-flashing always needs a USB cable.

**Read the Hardware and Safety sections below before wiring this up.**
This design regulates charge current by running an IRL540N MOSFET in its
*linear* region — it gets hot, on purpose, and needs a real heatsink.

## Files

- `car_battery_charger.ino` — the sketch.
- `config.h.example` — rename to `config.h` and fill in: device identity,
  hardware pins, INA219 calibration, and every charge-algorithm constant
  (voltages, current limits, timing). Keep `config.h` out of git.

## Before building

1. **Arduino IDE board package**: "esp32 by Espressif Systems", **core
   3.x or newer** (uses the new pin-based `ledcAttach()`/`ledcWrite()`
   LEDC API).
2. **Libraries** (Library Manager):
   - `Adafruit INA219` by Adafruit
   - `U8g2` by olikraus
   - `Adafruit NeoPixel` by Adafruit (only needed if your board has the
     onboard status LED and `STATUS_LED_ENABLED` is `true`)
   Built-in: `Preferences`, `Wire`.
3. Select board **"ESP32C3 Dev Module"**, and under Tools set **USB CDC
   On Boot: Enabled** (needed for Serial over native USB).
4. Rename `config.h.example` to `config.h` and fill in any pins that
   don't match your specific board's silkscreen (see Hardware below —
   the defaults are a reasonable starting point, not a guarantee for
   every "ESP32-C3-Zero" clone).

There's no OTA in this build — flash over USB every time.

## Hardware

### Pins (defaults — verify against your board's actual pinout diagram)

| Signal | Pin | Notes |
|---|---|---|
| I2C SDA | GPIO4 | shared bus: INA219 + OLED |
| I2C SCL | GPIO5 | |
| Encoder A | GPIO6 | interrupt-driven quadrature |
| Encoder B | GPIO7 | |
| Encoder button | GPIO3 | active low, internal pull-up |
| MOSFET gate (via RC filter) | GPIO1 | LEDC PWM, 20kHz carrier |
| Status LED (onboard WS2812) | GPIO10 | matches `smart_switch`'s convention |

Avoid the ESP32-C3 strapping pins (GPIO2, 8, 9) for anything with an
external pull resistor or load on it — GPIO9 is also the BOOT button on
most dev boards.

### MOSFET gate drive — read this before wiring

The IRL540N is run as a **linear** current-regulating element, not
switched fully on/off — there is no inductor or buck stage in this
design. The ESP32 PWMs its gate through an RC low-pass filter, turning
the fast PWM carrier into a smooth analog bias voltage that partially
enhances the MOSFET, so it behaves like a variable resistor:

```
MOSFET_GATE_PIN --[100R gate resistor]--+-- IRL540N gate
                                         |
                           [2.2k]--------+
                                         |
                 filter cap [1uF] -------+-- IRL540N source/GND
                                         |
                           [10k pulldown]+-- IRL540N source/GND
```

The 2.2k/1uF RC forms a ~72Hz low-pass — well below the 20kHz PWM
carrier (so ripple is filtered out) and well above the ~10Hz control-loop
update rate (so it still responds promptly). The 10k gate pulldown
guarantees the MOSFET stays OFF while the ESP32 is booting or if this
GPIO is ever left floating, before `setup()` configures it.

Power path: 26V DC supply → battery+ → battery− → MOSFET drain → MOSFET
source → GND, with the INA219 (and its 0.01Ω sense resistor, see below)
in series to measure charge current — either in the battery+ or
battery− leg, matching how your INA219 breakout's IN+/IN− are wired.
The firmware assumes current always flows one direction, into the
battery.

### Thermal warning — read this even if you skip everything else

At a 26V supply, the MOSFET dissipates
`(26V − battery_voltage) × charge_current` as heat, continuously. A
deeply discharged battery sitting at ~11.5V, charging at the 10A ceiling
this firmware now allows: `(26 − 11.5) × 10 ≈ 145W` — **continuously, in
a single TO-220 package.** That is beyond what one TO-220 IRL540N can
realistically shed even on a large finned heatsink with a fan (realistic
sustained dissipation for one TO-220 device on good cooling tops out
around 40–60W). To actually run near 10A safely you need either:

- multiple IRL540Ns in parallel sharing the load (with a gate resistor
  per device, not just tied gates), or
- accept that high current is only safe once the battery has already
  come up closer to `BULK_TARGET_VOLTAGE` (less voltage drop, less
  heat) — not at the start of a bulk charge on a deeply discharged
  battery, which is exactly when easy-mode picks the highest current.

The firmware has **no thermal sensor on the MOSFET** and cannot detect
it overheating. Watch the case temperature by hand (or an IR thermometer)
on first use at any current setting near the ceiling, and lower
`MAX_CHARGE_CURRENT_A` in `config.h` if it's climbing past what your
cooling can hold in steady state. `10.0f` is a sensor-range ceiling, not
a target to aim for.

### INA219 custom calibration (0.01Ω / 25W shunt, up to ~10A)

This build swaps the INA219 breakout's stock 0.1Ω shunt for a 0.01Ω/25W
resistor to extend the usable current range. The `Adafruit_INA219`
library only ships fixed calibration presets tuned for the stock 0.1Ω
shunt, so its `getCurrent_mA()` would silently read 10x too low with this
resistor — instead, the sketch writes the INA219's calibration register
directly and reads the raw current register itself, scaled by its own
`INA219_CURRENT_LSB_A`. Bus voltage (`getBusVoltage_V()`) is unaffected
by this and still uses the library normally.

Values used (see the derivation comment in `config.h.example` for the
full calculation): `INA219_CURRENT_LSB_A = 0.0004` (400µA/bit),
`INA219_CALIBRATION = 10240`, giving accurate readings up to ~13.1A
before the current register overflows — comfortable headroom above the
10A ceiling. If you use a different shunt value or want a different
range, recompute both constants together; they only make sense as a
matched pair.

## Charge algorithm

Three stages for flooded/AGM lead-acid, no continuous float:

1. **Bulk** — constant current at the selected target, until the battery
   reaches `BULK_TARGET_VOLTAGE` (14.4V default).
2. **Absorption** — constant voltage (`ABSORPTION_VOLTAGE`, 14.6V
   default), current tapers naturally as the battery fills.
3. **Done** — once current has stayed below `TERMINATION_CURRENT_FRACTION`
   (3%) of the selected capacity for `MIN_TERMINATION_DWELL_MS` (5 min)
   continuously, output stops entirely. Restarting always requires an
   explicit click — charging never auto-resumes after a reboot or power
   loss.

Safety nets that run independently of the control loop: hard
over-voltage cutoff (15V), hard over-current fault (125% of the current
limit, sustained), an absolute 14h max-duration timeout, and a "no
battery detected" refusal to start if the resting voltage isn't
plausible (below 8V).

State-of-charge is estimated from the battery's resting (open-circuit)
voltage right before a charge starts, then tracked forward by coulomb
counting (current × time, scaled by `CHARGE_EFFICIENCY` = 0.85) — this is
an estimate, not a lab-grade measurement. The OLED only shows a numeric
ETA during Bulk (constant current, so time-to-target is a straightforward
calculation); during Absorption it shows "Topping off..." instead of a
countdown, since the tapering current curve there isn't reliably
predictable.

## Local UI (rotary encoder)

- **Idle, manual mode**: rotate to adjust target charge current
  (`MANUAL_CURRENT_STEP_A` per detent), click to start charging at that
  current.
- **Idle, long-press**: enters easy mode — rotate to scroll through the
  capacity presets in `CAPACITY_PRESETS_AH` (44/50/55/60/65/70/80/85/
  90/100 Ah), click to confirm (sets target current to
  `BULK_CURRENT_FRACTION_OF_CAPACITY` × capacity, i.e. C/10 by default —
  10A for the 100Ah preset, see the thermal warning above), long-press
  again to cancel back out without changing anything.
- **While charging**: click stops immediately and returns to idle.
  Rotation and long-press are ignored while active — stop first to
  change settings.
- **Done / Fault**: click acknowledges and returns to idle.

The onboard status LED (if present/enabled) mirrors state at a glance:
blue = idle, yellow = bulk, orange = absorption, green = done, red =
fault.

## Tuning the control loop

Open the Serial Monitor at 115200 baud while charging — the control loop
logs `state`, voltage, current, target current, and gate duty once per
second. `BULK_KP`/`BULK_KI`/`ABS_KP`/`ABS_KI` in `config.h` are starting
points, not calibrated for your exact RC filter/MOSFET sample: if current
or voltage oscillates, lower the Kp for that stage; if it settles too
slowly, raise Ki a little.

## Bluetooth / companion app

The ESP32-C3 has BLE, but this build has no networking or remote control
at all — the encoder and OLED are the only interface. If a phone app is
wanted later, the natural next step is adding a BLE GATT service
alongside the existing control loop.

## A note on verification

I cross-checked the `Adafruit_INA219` and `U8g2` API calls used here
against their published APIs, the INA219 custom-calibration register
writes against the INA219 datasheet's calibration procedure, and the
control-loop/state-machine logic against standard 3-stage lead-acid
charging practice. I could not compile this sketch or run it on real
hardware in the sandbox that generated it — no ESP32 toolchain or
physical charger available there. Before trusting this on an actual
battery: build it, watch the Serial Monitor through a full
bulk→absorption→done cycle, and separately verify with a multimeter (and
a known-accurate ammeter, given the custom shunt) that measured
voltage/current match the OLED and Serial readings before leaving it
unattended.

## Version History

| Version | Date | Changes |
|---|---|---|
| v1.0.0 | 2026-09-18 | Initial firmware: 3-stage (bulk/absorption/done) lead-acid charging via linear-region IRL540N control, INA219 sensing, SH1106 OLED status/progress display, rotary-encoder local UI with easy-mode capacity presets, MQTT + Home Assistant discovery, ArduinoOTA. |
| v1.1.0 | 2026-09-18 | Removed MQTT/Home Assistant connectivity and ArduinoOTA entirely -- standalone local-only device now (encoder + OLED only), no WiFi. Added a custom INA219 calibration for a 0.01 ohm/25W shunt resistor (replacing the breakout's stock 0.1 ohm shunt), raising accurate current range to ~13A and `MAX_CHARGE_CURRENT_A` to 10A; added a throttled Serial tuning log to the control loop. |
