# Car Battery Charger — ESP32-C3-Zero smart lead-acid charger

Standalone smart charger for flooded/AGM 12V car batteries (44-100Ah).
No WiFi/MQTT/cloud/OTA — everything is local: an INA219 current sensor,
a second ADC-based supply-voltage sense, a SH1106 128x64 OLED, a rotary
encoder with push button, and a piezo buzzer are the entire interface.
Re-flashing always needs a USB cable.

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
   - `Adafruit NeoPixel` by Adafruit (needed if `STATUS_LED_ENABLED` is
     `true`, the default -- **if you skip this, the status LED silently
     does nothing.** `__has_include()` compiles the whole NeoPixel code
     path out when the library isn't found, with no compile error and no
     other symptom. The Serial Monitor prints which way it went at boot
     -- "status LED initialized on GPIOx" or a library-missing warning --
     check that first if the LED never lights up.)
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
| Encoder A | GPIO6 | interrupt-driven quadrature, bounce-rejecting |
| Encoder B | GPIO7 | also interrupt-driven -- see below |
| Encoder button | GPIO3 | active low, internal pull-up |
| MOSFET gate bias (via RC filter, feeds all 4 gates) | GPIO1 | LEDC PWM, 20kHz carrier |
| Supply-rail voltage divider | GPIO0 | ADC1_CH0, see Voltage sensing below |
| Status LED (onboard WS2812) | GPIO10 | matches `smart_switch`'s convention |
| Piezo buzzer | GPIO8 | see note below — this is a strapping pin |

Every non-strapping GPIO is already spoken for above, so the buzzer
defaults to GPIO8 — a strapping pin, but one only sampled at boot/reset;
this sketch never drives it until well after boot, and a bare piezo (no
extra pull resistor of your own) shouldn't disturb that sampling. Verify
your specific board doesn't already use GPIO8 for something else first
(a few ESP32-C3 dev boards put a second onboard LED there) — GPIO2 is
the fallback if so. Avoid GPIO9 for anything external — it's the BOOT
button on most of these boards.

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

### Piezo buzzer

A passive piezo on `BUZZER_PIN`, driven via the ESP32 core's
`tone()`/`noTone()` — it manages its own LEDC channel internally, so it
doesn't conflict with the MOSFET gate's own `ledcAttach()` on a
different pin. Wire it straight off the GPIO through a small series
resistor (100Ω, protects the pin from the piezo's capacitive inrush).
For louder output than a bare piezo gives at 3.3V, drive it through a
small NPN transistor instead (GPIO → base resistor, piezo + resistor
from 3V3/5V → collector, emitter → GND) — optional, not required for a
basic audible alert.

It plays a short, non-blocking tone pattern (`updateBuzzer()`, called
every `loop()` iteration, steps through the pattern by elapsed time —
never a `delay()`, so it can't stall the control loop or encoder) on
every charge state transition:

- **Charging starts** — a short rising chirp.
- **Charge completes** — a rising triple beep.
- **Fault, including a refused start** (no battery detected, voltage
  already too high, over-voltage/over-current/timeout) — a high-pitched
  triple beep.
- **Manual stop** — a single neutral click.

Patterns are plain frequency/duration arrays in `config.h` (`BUZZER_START_*`,
`BUZZER_DONE_*`, `BUZZER_FAULT_*`, `BUZZER_STOP_*`) if you want to change
the tones. Set `BUZZER_ENABLED = false` to disable it entirely.

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

**Decoding**: both A and B are interrupt-driven (not just A), and every
edge is run through a full quadrature state table (`onEncoderChange()`)
that only accepts legal single-step transitions along the Gray-code
sequence and silently drops everything else — including mechanical
contact bounce, which is what a naive "compare A and B on every A edge"
decoder has no way to distinguish from a real click. If it still feels
jumpy after this, it's likely a genuinely noisy encoder and worth adding
a small hardware debounce (100nF from A to GND and from B to GND).

**Direction**: `ENCODER_REVERSED` in `config.h` flips which physical
rotation direction increases a value — there's no universal "clockwise
means more" convention, it depends on which of A/B you wired where.
Flip it (or swap the A/B wires at the encoder, which does the same
thing) if clockwise decreases instead of increases.

No separate picker screen or confirm step — rotating applies the value
live, and a full-screen readout pops up while you do it (see "Big
overlay" below).

- **Idle, manual mode**: rotate to adjust target charge current
  (`MANUAL_CURRENT_STEP_A` per detent) — takes effect immediately.
  Click to start charging at that current.
- **Idle, easy mode**: rotate to step through the capacity presets in
  `CAPACITY_PRESETS_AH` (44/50/55/60/65/70/80/85/90/100 Ah) — each one
  sets target current to `BULK_CURRENT_FRACTION_OF_CAPACITY` × capacity
  (C/10 by default — 10A for the 100Ah preset, see the thermal warning
  above), applied immediately as you turn it. Click to start charging.
- **Idle, long-press**: instantly toggles between manual and easy mode
  (using whichever capacity preset was last selected). No picker, no
  confirm — the mode just switches, and the readout confirms which one
  you're in now.
- **While charging**: click stops immediately and returns to idle.
  Rotation and long-press are ignored while active — stop first to
  change settings.
- **Done / Fault**: click acknowledges and returns to idle.

The onboard status LED (if present/enabled) mirrors state at a glance:
blue = idle, yellow = bulk, orange = absorption, green = done, red =
fault. The buzzer backs this up audibly — see "Piezo buzzer" above — so
a state change registers even if you're not looking at the device.

Uses `NEO_RGB` color order, not the more common `NEO_GRB` — that's what
Waveshare's own docs specify for the ESP32-C3-Zero's onboard WS2812. If
you're driving a different/external WS2812 instead, it may need
`NEO_GRB` back.

### Persisted settings

Mode (manual/easy), the selected capacity preset, and the manual target
current are saved to NVS (via `Preferences`) whenever they change, and
restored at boot — the device comes back up the way you left it rather
than resetting to `config.h`'s defaults every power cycle. The Serial
Monitor logs what it loaded at boot (`[nvs] loaded: ...`), and logs a
clear failure message on either load or save if the NVS namespace can't
be opened, rather than settings just silently failing to stick.

Charging progress itself is the one thing that's deliberately **not**
persisted — a fresh boot always comes up `Idle` regardless of what was
happening before, and starting a charge always requires an explicit
click. See "Charge algorithm" above for why.

## Display

The normal status screen is four bands, top to bottom: state name
(bold) with a capacity/manual badge on the right; a battery icon with
SoC% and a small dot that only appears while actively charging; a big
voltage/current readout; and a context line (ETA during bulk, "Topping
off..." during absorption, the fault reason, or the target current
while idle).

That bottom line — and the fault reason, which is also free-form text —
is drawn through `drawStrFit()`, which measures the string in the
current font and truncates it with "..." rather than letting it
silently run off the right edge of the display.

### Big overlay

Rotating the encoder or long-pressing while idle replaces the whole
screen with a full-screen readout (`drawBigOverlay()`) instead of the
normal status screen — a one-line label ("Battery capacity" or "Manual
current") plus the value itself in the biggest font that still fits
without clipping (it tries 32pt, then 24pt, falling back to the 16pt
font used elsewhere on this display, which is guaranteed to fit any
value this ever shows). It auto-dismisses back to the normal screen
`BIG_OVERLAY_SCROLL_MS` (3s default) after the last encoder tick, or
`BIG_OVERLAY_MODE_SWITCH_MS` (2s default) after a mode-switch
long-press — both in `config.h`. The underlying value applies live as
you turn it; the overlay is purely a display concern; the click button
starts/stops charging exactly the same whether or not it's showing.

### Fault screen

A fault (`drawFaultScreen()`) also takes over the whole display, the
same way the big overlay does — "FAULT" itself as big as fits (same
biggest-font-that-fits logic, via the shared `drawBigCentered()`
helper), with the actual reason in small text at the bottom
(`drawStrFit()`, since fault reasons vary in length and several don't
fit this font at full width). Unlike the value overlay, this doesn't
auto-dismiss — it stays up until you click to acknowledge, same as
before.
Text that's too long for a 128px-wide display isn't a hypothetical: a
fixed-length assumption is exactly what let `"(click=start)"` and a
couple of the longer fault-reason strings get cut off mid-word before.

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

One specific thing I couldn't verify: `drawBigOverlay()` uses
`u8g2_font_logisoso32_tf` and `u8g2_font_logisoso24_tf` for the
full-screen readout, on top of the `u8g2_font_logisoso16_tf` already
used elsewhere and confirmed working on your build. I'm confident the
logisoso family includes these sizes, but haven't compiled against them
specifically — if either name doesn't exist in your installed U8g2
version, the compiler error will name the missing font; check
[U8g2's font list](https://github.com/olikraus/u8g2/wiki/fntlistall)
for the closest available size in that family and swap it into the
`bigFonts[]` array.

## Version History

| Version | Date | Changes |
|---|---|---|
| v1.0.0 | 2026-09-18 | Initial firmware: 3-stage (bulk/absorption/done) lead-acid charging via linear-region IRL540N control, INA219 sensing, SH1106 OLED status/progress display, rotary-encoder local UI with easy-mode capacity presets, MQTT + Home Assistant discovery, ArduinoOTA. |
| v1.1.0 | 2026-09-18 | Removed MQTT/Home Assistant connectivity and ArduinoOTA entirely -- standalone local-only device now (encoder + OLED only), no WiFi. Added a custom INA219 calibration for a 0.01 ohm/25W shunt resistor (replacing the breakout's stock 0.1 ohm shunt), raising accurate current range to ~13A and `MAX_CHARGE_CURRENT_A` to 10A; added a throttled Serial tuning log to the control loop. |
| v1.2.0 | 2026-09-18 | Fixed a battery-voltage measurement bug: the INA219's bus-voltage reading is the floating MOSFET-side return rail, not battery voltage, given the low-side MOSFET placement -- added a second resistor divider into an ESP32 ADC pin (GPIO0) to measure the actual supply rail, and compute battery voltage as supply_measured minus the INA219 bus reading. Also moved the MOSFET stage to a bank of 4 parallel IRL540Ns with per-device source ballast resistors for current sharing, spreading the ~145W worst-case dissipation at the 10A ceiling across multiple packages. |
| v1.3.0 | 2026-09-18 | Added a piezo buzzer (GPIO8) with a non-blocking tone sequencer (`updateBuzzer()`): distinct patterns for charge start, charge complete, fault/refused start, and manual stop. |
| v1.3.1 | 2026-09-19 | Fixed a compile error: `Adafruit_INA219.h` already `#define`s `INA219_REG_CALIBRATION`/`INA219_REG_CURRENT` itself, and this sketch's own `static const` declarations of the same names collided with those macros at preprocessing. Removed the redundant declarations; the register writes now just use the library's own macros directly. |
| v1.4.0 | 2026-09-19 | Fixed encoder bouncing/jumping: replaced the naive "compare A and B on every A edge" decoder with a full quadrature state-table decoder driven by interrupts on both A and B, which structurally rejects mechanical contact bounce instead of counting every raw edge. Added `ENCODER_REVERSED` to flip rotation direction (clockwise was decreasing current; defaults to `true` now so clockwise increases it). |
| v1.5.0 | 2026-09-19 | Fixed the status LED: wrong color order (`NEO_GRB` instead of the `NEO_RGB` Waveshare's own docs specify for the ESP32-C3-Zero's onboard WS2812) and added a boot-time Serial message so a missing `Adafruit_NeoPixel` library -- which silently compiles the whole LED code path out via `__has_include()`, with no other symptom -- is diagnosable instead of just "the LED does nothing." Redesigned the OLED layout: battery icon with SoC%, bold header with a capacity/manual badge, bigger voltage/current readout, and a `drawStrFit()` helper that measures and truncates any status text instead of letting it silently run off the display edge (fixes `"(click=start)"` and several fault-reason strings getting cut off). |
| v1.6.0 | 2026-09-19 | `ENCODER_REVERSED` default flipped to `false` (confirmed correct on the actual build). Fixed the capacity-select screen's header text overflowing the display uncaught (`u8g2.drawStr()` instead of `drawStrFit()`) and shortened the confirm/cancel hint so it fits without truncating mid-word. Added a "Manual" entry past the last capacity preset in the picker -- previously, once a capacity was confirmed there was no way back to manual current entry at all; scroll one past the last preset and click to get back. |
| v1.6.1 | 2026-09-19 | Widened the battery icon roughly 4x (20px to 86px body width) for a more readable charge-progress indicator; `drawBatteryIcon()` now takes explicit width/height parameters instead of hardcoded dimensions. Repositioned the SoC% text and charging-indicator dot to make room. |
| v1.7.0 | 2026-09-19 | Replaced the picker-with-confirm capacity/mode UI entirely: rotating the encoder now applies the value live (no confirm step) and shows a full-screen readout (`drawBigOverlay()`, biggest font that fits) that auto-dismisses 3s after the last tick; long-press instantly toggles manual/easy mode instead of opening a picker, showing the same readout for 2s. Removed `UiMode`/`uiMode` and the capacity-select screen entirely -- this also structurally closes the "no way to exit" class of bug from the old modal picker, since there's no longer a separate mode to get stuck in. Added `BIG_OVERLAY_SCROLL_MS`/`BIG_OVERLAY_MODE_SWITCH_MS`. |
| v1.8.0 | 2026-09-19 | Fixed persisted settings (mode/capacity/target current) never actually saving: `DEVICE_ID` ("car_battery_charger1", 20 chars) exceeded ESP-IDF's 15-character NVS namespace limit, so every `prefs.begin()` call was failing -- silently, since the return value was never checked. Shortened `DEVICE_ID` to `"cbc1"` and added error checking with a Serial Monitor message on both load and save, so a namespace failure is diagnosable instead of settings just quietly resetting to defaults every boot. |
| v1.9.0 | 2026-09-21 | Fault now takes over the whole display (`drawFaultScreen()`) instead of sharing the normal 4-band status screen: "FAULT" as big as fits, fault reason in small text at the bottom. Extracted the biggest-font-that-fits logic from `drawBigOverlay()` into a shared `drawBigCentered()` helper used by both. |
| v1.9.1 | 2026-09-21 | Fixed the fault screen's big "FAULT" text overlapping the reason text below it -- the 32pt font's baseline (58) put its glyph body directly on top of the fixed reason line at y=62, since a font that tall reaches ~32px above its own baseline. Raised all three `drawBigCentered()` baselines (48/42/34) for proper clearance. |
| v1.9.2 | 2026-09-21 | v1.9.1 raised the shared baseline table, which regressed the capacity/current overlay's positioning too -- it never had an overlap problem, only the fault screen did. `drawBigCentered()` now takes its baseline array as a parameter instead of a hardcoded shared one; the overlay keeps its original {58,50,40}, the fault screen keeps the raised {48,42,34}. |
