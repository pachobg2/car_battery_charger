# Car Battery Charger — ESP32-C3-Zero smart lead-acid charger

Standalone smart charger for flooded/AGM 12V car batteries (44-100Ah).
No WiFi/MQTT/cloud/OTA — everything is local: an INA219 current sensor,
a second ADC-based supply-voltage sense, a SH1106 128x64 OLED, and a
rotary encoder with push button are the entire interface. Re-flashing
always needs a USB cable.

**Read the Hardware section below before wiring this up.** This design
regulates charge current by running a bank of 4 parallel IRL540N MOSFETs
in their *linear* region — they get hot, on purpose, and need real
heatsinking. The voltage-sensing topology is also less obvious than it
looks; see "Voltage sensing" below before you assume the INA219 reads
battery voltage directly.

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
| MOSFET gate bias (via RC filter, feeds all 4 gates) | GPIO1 | LEDC PWM, 20kHz carrier |
| Supply-rail voltage divider | GPIO0 | ADC1_CH0, see Voltage sensing below |
| Status LED (onboard WS2812) | GPIO10 | matches `smart_switch`'s convention |

Avoid the ESP32-C3 strapping pins (GPIO2, 8, 9) for anything with an
external pull resistor or load on it — GPIO9 is also the BOOT button on
most dev boards.

### Voltage sensing — read this before wiring, it changes the topology

Battery voltage is **not** read from the INA219's bus-voltage register.
The MOSFET bank has to sit in the battery's negative return leg (so its
gates can be driven straight from a ground-referenced ESP32 GPIO — see
below), which means the node between the battery's negative terminal and
the MOSFETs actually floats: as the battery charges from ~11.5V to
~14.6V, that node's voltage *relative to true system ground* falls from
~14.5V to ~11.4V (it's the supply voltage minus the battery voltage).
The INA219's bus-voltage reading is always relative to its own GND pin —
which has to share system ground with the ESP32 for I2C to work — so
`getBusVoltage_V()` reads that floating return node, not battery voltage.

The fix: a second, independent resistor divider (`SUPPLY+ --[110k]--
ADC pin --[10k]-- GND`, plus a 100nF cap to ground) feeds ESP32 ADC1_CH0
(GPIO0), measuring the actual supply rail. Firmware computes:

```
battery_voltage = supply_rail_measured − ina219_bus_reading
```

Both terms are ground-referenced, so this stays accurate even if the 26V
supply sags under load or isn't exactly 26V — which mattered enough to
fix, since the charge algorithm's thresholds are 0.2V apart (14.4V vs
14.6V). `SUPPLY_DIVIDER_RATIO` in `config.h` (12.0 nominal) should be
calibrated against a multimeter reading on the real supply rail; the
Serial Monitor prints the measured rail voltage at boot.

The INA219 itself is used **only for current sensing** now (via its
shunt, using the custom calibration described below) — its bus-voltage
feature plays no role in the charge algorithm.

### MOSFET bank — 4x IRL540N in parallel, current-shared

Run in their **linear** region as variable resistors, not switched fully
on/off — there's no inductor or buck stage in this design. One shared
gate-bias network drives all four (purely analog — no firmware change
needed to add MOSFETs):

```
MOSFET_GATE_PIN --[2.2k]--+------------------------------ BIAS node
                          |
                      [1uF cap]
                          |
                         GND
BIAS --[10k pulldown]-- GND
BIAS --[100R]-- Q1 gate      BIAS --[100R]-- Q2 gate
BIAS --[100R]-- Q3 gate      BIAS --[100R]-- Q4 gate
```

The 2.2k/1uF RC forms a ~72Hz low-pass — well below the 20kHz PWM
carrier (so ripple is filtered out) and well above the ~10Hz control-loop
update rate (so it still responds promptly). The 10k pulldown guarantees
every gate sits at 0V while the ESP32 boots or if the GPIO floats, before
`setup()` configures it. Each MOSFET gets its own 100R gate-stopper
resistor off the shared BIAS node — tying paralleled gates straight
together invites high-frequency parasitic oscillation between devices.

**Current sharing matters here, and it's not automatic.** MOSFETs in
their linear region don't share current evenly on their own: a device
running slightly hotter conducts *more* current at the same Vgs (its
threshold voltage drops with temperature), which heats it further —
thermal runaway that can destroy one MOSFET while its neighbors barely
warm up. Each MOSFET's source gets its own small ballast resistor before
the common return:

```
Q1 source --[0.22R 5W]--+
Q2 source --[0.22R 5W]--+-- common return -- 0.01R main shunt -- GND
Q3 source --[0.22R 5W]--+
Q4 source --[0.22R 5W]--+
```

A device pulling more than its share drops more voltage across its own
ballast resistor, which lowers *its own* effective Vgs (gate is shared,
source rises) and throttles it back — self-correcting negative feedback.
At ~2.5A/device (10A ÷ 4), 0.22Ω drops ~0.55V (real balancing authority)
and dissipates ~1.4W (use 5W resistors for margin). Matched MOSFETs from
the same batch help too — less imbalance for the ballast resistors to
correct.

Power path: 26V DC supply → battery+ → battery− → the four MOSFET
drains (paralleled) → each MOSFET's own ballast resistor → common
return → the 0.01Ω/25W main current shunt → supply/system ground. The
INA219's VIN+/VIN− sense directly across that main shunt's two leads
(Kelvin-style — wire the sense leads to the resistor's own terminals,
not through the high-current power lugs, or shunt-lead resistance will
skew the reading).

### Thermal warning — read this even if you skip everything else

Total dissipation across the whole bank is still
`(supply_voltage − battery_voltage) × charge_current`, same physics as a
single device — paralleling only **spreads** that heat, it doesn't
reduce it. A deeply discharged battery sitting at ~11.5V, charging at
the 10A ceiling this firmware allows: `(26 − 11.5) × 10 ≈ 145W` total —
about **36W per MOSFET** across the 4-way bank, worst case, right at the
start of a bulk charge (exactly when easy-mode picks the highest
current). 36W per TO-220 is realistic on a real heatsink with airflow,
but it's not casual — this is not a "bolt on any old heatsink" build.

The firmware has **no thermal sensor** on the MOSFETs and cannot detect
overheating. Watch case temperature by hand (or an IR thermometer) on
first use anywhere near the current ceiling, and back `MAX_CHARGE_CURRENT_A`
off — or add more parallel MOSFETs — if it's climbing past what your
cooling holds in steady state. If you build with fewer than 4 MOSFETs,
lower `MAX_CHARGE_CURRENT_A` proportionally in `config.h`.

### INA219 custom calibration (0.01Ω / 25W shunt, up to ~10A)

This build swaps the INA219 breakout's stock 0.1Ω shunt for a 0.01Ω/25W
resistor to extend the usable current range. The `Adafruit_INA219`
library only ships fixed calibration presets tuned for the stock 0.1Ω
shunt, so its `getCurrent_mA()` would silently read 10x too low with this
resistor — instead, the sketch writes the INA219's calibration register
directly and reads the raw current register itself, scaled by its own
`INA219_CURRENT_LSB_A`.

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
| v1.2.0 | 2026-09-18 | Fixed a battery-voltage measurement bug: the INA219's bus-voltage reading is the floating MOSFET-side return rail, not battery voltage, given the low-side MOSFET placement -- added a second resistor divider into an ESP32 ADC pin (GPIO0) to measure the actual supply rail, and compute battery voltage as supply_measured minus the INA219 bus reading. Also moved the MOSFET stage to a bank of 4 parallel IRL540Ns with per-device source ballast resistors for current sharing, spreading the ~145W worst-case dissipation at the 10A ceiling across multiple packages. |
