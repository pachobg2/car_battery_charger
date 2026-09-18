# Car Battery Charger — ESP32-C3-Zero smart lead-acid charger

Raw Arduino C++ smart charger for flooded/AGM 12V car batteries (44-100Ah),
following the same house pattern as the rest of this fleet: `espMqttClient`
with QoS 1, non-blocking WiFi/MQTT reconnect, LWT availability, retained HA
MQTT discovery, `ArduinoOTA`, manufacturer `P@cho`. Adds local control via
an INA219 current/voltage sensor, a SH1106 128x64 OLED, and a rotary
encoder with push button.

**Read the Hardware and Safety sections below before wiring this up.**
This design regulates charge current by running an IRL540N MOSFET in its
*linear* region — it gets hot, on purpose, and needs a real heatsink.

## Files

- `car_battery_charger.ino` — the sketch.
- `config.h.example` — rename to `config.h` and fill in: WiFi/MQTT/OTA
  credentials, device identity, hardware pins, and every charge-algorithm
  constant (voltages, current limits, timing). Keep `config.h` out of git.

## Before building

1. **Arduino IDE board package**: "esp32 by Espressif Systems", **core
   3.x or newer** (uses the new pin-based `ledcAttach()`/`ledcWrite()`
   LEDC API).
2. **Libraries** (Library Manager):
   - `espMqttClient` by bertmelis
   - `Adafruit INA219` by Adafruit
   - `U8g2` by olikraus
   - `Adafruit NeoPixel` by Adafruit (only needed if your board has the
     onboard status LED and `STATUS_LED_ENABLED` is `true`)
   Built-in: `WiFi`, `ArduinoOTA`, `Preferences`, `Wire`.
3. Select board **"ESP32C3 Dev Module"**, and under Tools set **USB CDC
   On Boot: Enabled** (needed for Serial over native USB).
4. Rename `config.h.example` to `config.h` and fill in your WiFi/MQTT
   credentials and any pins that don't match your specific board's
   silkscreen (see Hardware below — the defaults are a reasonable
   starting point, not a guarantee for every "ESP32-C3-Zero" clone).
5. DHCP only, no static IP in firmware — same reasoning as the rest of
   the fleet. Set a DHCP reservation on your router if you want a stable
   address.

## First flash vs. later updates

First flash needs a USB cable. After that, `ArduinoOTA` exposes the board
as a network port in `Tools > Port`, protected by `OTA_PASSWORD`. On
every OTA start the firmware forces the gate duty to 0 first — it will
never leave the MOSFET driving current mid-flash.

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
source → GND, with the INA219 in series to measure charge current (either
in the battery+ or battery− leg, matching how your INA219 breakout's
IN+/IN− are wired — the firmware assumes current always flows one
direction, into the battery).

**Thermal warning:** at a 26V supply, the MOSFET dissipates
`(26V − battery_voltage) × charge_current` as heat, continuously — e.g.
about 54W at 4A into a 12.6V battery. `MAX_CHARGE_CURRENT_A` in
`config.h` defaults to a conservative 3.0A, but even that is real,
sustained heat (roughly 40W at a 12.6V battery). **Put this MOSFET on a
proper heatsink sized for your actual current, with airflow if you push
it.** The firmware has no thermal sensor on the MOSFET and cannot detect
it overheating — that protection is entirely up to your heatsink.

### Why the current ceiling is 3.0A, not higher

This sketch uses `Adafruit_INA219`'s default calibration
(`setCalibration_32V2A()`, set inside `ina219.begin()`), which on a
standard 0.1Ω shunt breakout only reads current accurately up to about
3.2A — above that the current register saturates and readings (which
both the control loop and the over-current safety check depend on)
silently stop being trustworthy right where it matters most. Don't raise
`MAX_CHARGE_CURRENT_A` past ~3.0A unless you also add a custom INA219
calibration (a smaller shunt resistor plus a matching `setCalibration()`
call) for real higher-current range.

## Charge algorithm

Three stages for flooded/AGM lead-acid, no continuous float:

1. **Bulk** — constant current at the selected target, until the battery
   reaches `BULK_TARGET_VOLTAGE` (14.4V default).
2. **Absorption** — constant voltage (`ABSORPTION_VOLTAGE`, 14.6V
   default), current tapers naturally as the battery fills.
3. **Done** — once current has stayed below `TERMINATION_CURRENT_FRACTION`
   (3%) of the selected capacity for `MIN_TERMINATION_DWELL_MS` (5 min)
   continuously, output stops entirely. Restarting always requires an
   explicit click/command — charging never auto-resumes after a reboot
   or power loss.

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
  `BULK_CURRENT_FRACTION_OF_CAPACITY` × capacity, i.e. C/10 by default),
  long-press again to cancel back out without changing anything.
- **While charging**: click stops immediately and returns to idle.
  Rotation and long-press are ignored while active — stop first to
  change settings.
- **Done / Fault**: click acknowledges and returns to idle.

The onboard status LED (if present/enabled) mirrors state at a glance:
blue = idle, yellow = bulk, orange = absorption, green = done, red =
fault.

## MQTT / Home Assistant

Everything on the local UI is mirrored over MQTT, so the charger can also
be watched/driven remotely:

- Sensors: charge state, battery voltage, charge current, state of
  charge %, Ah delivered, fault reason, plus the usual diagnostics (WiFi
  RSSI, reset reason, MQTT fail count, uptime).
- `Target Current` number entity and `Battery Capacity` select entity
  (options: `manual` plus each Ah preset) — both are rejected while
  actively charging; stop first.
- `Start Charging` / `Stop Charging` buttons.

Retained HA discovery configs are published on every broker connect,
bundling all of the above under one device named "Car Battery Charger".

## Bluetooth / companion app

The ESP32-C3 has BLE, but there's no companion app in this repo yet —
MQTT/Home Assistant is the remote-control channel for now. If a
dedicated phone app is wanted later, the natural next step is a BLE GATT
service alongside the existing MQTT client (not a replacement for it).

## A note on verification

I cross-checked the `espMqttClient`, `Adafruit_INA219`, and `U8g2` API
calls used here against their published APIs, and the control-loop/state-
machine logic against standard 3-stage lead-acid charging practice. I
could not compile this sketch or run it on real hardware in the sandbox
that generated it — no ESP32 toolchain or physical charger available
there. Before trusting this on an actual battery: build it, watch the
Serial Monitor through a full bulk→absorption→done cycle, and separately
verify with a multimeter that measured voltage/current match the OLED
and MQTT readings before leaving it unattended. The PI control gains
(`BULK_KP`/`BULK_KI`/`ABS_KP`/`ABS_KI` in `config.h`) are starting points
for tuning, not a calibrated match for your exact RC filter and MOSFET
sample — if current oscillates, lower the Kp; if it settles too slowly,
raise Ki a little.

## Version History

| Version | Date | Changes |
|---|---|---|
| v1.0.0 | 2026-09-18 | Initial firmware: 3-stage (bulk/absorption/done) lead-acid charging via linear-region IRL540N control, INA219 sensing, SH1106 OLED status/progress display, rotary-encoder local UI with easy-mode capacity presets, MQTT + Home Assistant discovery, ArduinoOTA. |
