/*
 * Car Battery Charger — ESP32-C3-Zero smart lead-acid charger
 *
 * 3-stage (bulk / absorption / done) charger for flooded/AGM car
 * batteries, built around:
 *   - INA219 (I2C) for charge current sensing, with a CUSTOM calibration
 *     for a 0.01 ohm / 25W shunt resistor (swapped in place of the
 *     breakout's stock 0.1 ohm shunt) for range up to ~10A. See
 *     config.h.example for how INA219_CALIBRATION/INA219_CURRENT_LSB_A
 *     were computed, and the thermal warning on MAX_CHARGE_CURRENT_A.
 *   - Battery VOLTAGE is NOT read from the INA219's bus-voltage register
 *     -- the MOSFET bank's low-side placement (see below) means that
 *     reading is the floating battery-negative return rail, not battery
 *     voltage. Instead a second resistor divider into an ESP32 ADC pin
 *     measures the supply rail, and battery_voltage = supply_measured -
 *     ina219_bus_reading (see readBatteryVoltage() and the long comment
 *     on SUPPLY_VOLTAGE_DIVIDER_PIN in config.h.example).
 *   - SH1106 128x64 OLED (I2C, via U8g2) for status/progress display
 *   - Rotary encoder with push button for local control
 *   - A bank of 4 parallel IRL540Ns (Q1-Q4), run in their LINEAR region
 *     (PWM through an RC filter onto a shared gate bias) as the
 *     current-regulating element -- there is no inductor/buck stage in
 *     this design. Paralleled to split the heat one MOSFET can't handle
 *     alone at this current; each has its own source ballast resistor
 *     for current sharing (linear-mode paralleling needs this -- see
 *     config.h.example for why). See config.h.example for the full gate
 *     drive schematic and an important thermal warning either way: this
 *     bank dissipates real, continuous heat and needs proper heatsinking.
 *
 * Standalone device -- no WiFi/MQTT/Home Assistant/OTA. Everything is
 * local: the OLED + rotary encoder are the only interface. Re-flashing
 * always needs a USB cable.
 *
 * Charge algorithm (flooded/AGM lead-acid):
 *   BULK: constant current (targetCurrentA) until battery reaches
 *         BULK_TARGET_VOLTAGE.
 *   ABSORPTION: constant voltage (ABSORPTION_VOLTAGE), current tapers
 *         naturally as the battery fills.
 *   DONE: once current has stayed below TERMINATION_CURRENT_FRACTION of
 *         the selected capacity for MIN_TERMINATION_DWELL_MS, output is
 *         stopped entirely (no continuous float stage). Restarting
 *         requires an explicit click.
 * Safety nets independent of the control loop: hard over-voltage cutoff,
 * hard over-current fault, absolute max charge duration, and a "no
 * battery detected" refusal to start if the sensed voltage isn't
 * plausible. Charging never auto-resumes after a reboot/power loss --
 * always requires a fresh explicit start.
 *
 * Local UI: rotating the encoder while idle adjusts the live value --
 * manual target current, or the capacity preset (from config.h's
 * CAPACITY_PRESETS_AH, auto-computing bulk current via
 * BULK_CURRENT_FRACTION_OF_CAPACITY) if in easy mode -- and pops up a
 * full-screen readout for a few seconds (see drawBigOverlay()). No
 * separate confirm step; it applies as you turn it. Long-press instantly
 * toggles between manual and easy mode, showing the same readout
 * briefly. Click still just starts/stops/acknowledges, unchanged by any
 * of this.
 *
 * A piezo buzzer (BUZZER_PIN, driven via tone()/noTone()) plays a short
 * non-blocking tone pattern on every charge state transition: a rising
 * chirp on start, a rising triple beep on a completed charge, a low
 * triple beep on any fault (including a refused start), and a single
 * neutral click on a manual stop. See updateBuzzer() below and the
 * BUZZER_* patterns in config.h.example.
 *
 * Libraries needed (Arduino IDE > Library Manager):
 *   - Adafruit INA219 (adafruit) -- only used for begin()/getBusVoltage_V();
 *     current is read via a custom register write/read, see below.
 *   - U8g2 (olikraus)
 *   - Adafruit NeoPixel (adafruit) -- only if STATUS_LED_ENABLED
 *   Built-in / come with the ESP32 Arduino core: Preferences, Wire
 *
 * Board package: esp32 by Espressif Systems -- board "ESP32C3 Dev
 * Module". Rename config.h.example -> config.h and fill in your hardware
 * pins/algorithm constants before building.
 */

#include <Preferences.h>
#include <esp_system.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <U8g2lib.h>
#if __has_include(<Adafruit_NeoPixel.h>)
#include <Adafruit_NeoPixel.h>
#endif

#include "config.h"

// ------------------------------------------------------------------
// Charge state machine
// ------------------------------------------------------------------
enum ChargeState { STATE_IDLE, STATE_BULK, STATE_ABSORPTION, STATE_DONE, STATE_FAULT };

const char* chargeStateName(ChargeState s) {
  switch (s) {
    case STATE_IDLE:       return "Idle";
    case STATE_BULK:       return "Bulk";
    case STATE_ABSORPTION: return "Absorption";
    case STATE_DONE:       return "Done";
    case STATE_FAULT:      return "Fault";
    default:               return "?";
  }
}

// ------------------------------------------------------------------
// Globals
// ------------------------------------------------------------------
Preferences prefs;
Adafruit_INA219 ina219(INA219_I2C_ADDR);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /*reset=*/U8X8_PIN_NONE);
#if __has_include(<Adafruit_NeoPixel.h>)
// NEO_RGB, not NEO_GRB -- Waveshare's own docs for the ESP32-C3-Zero
// specify RGB color order for its onboard WS2812, not the more common
// GRB. Wrong order gives wrong colors, not a dead LED, but it's wrong.
Adafruit_NeoPixel statusLed(1, STATUS_LED_PIN, NEO_RGB + NEO_KHZ800);
#endif

ChargeState chargeState = STATE_IDLE;
String faultReason = "";

bool useEasyMode = false;
uint8_t capacityPresetIndex = DEFAULT_CAPACITY_PRESET_INDEX;
uint16_t selectedCapacityAh = 0; // 0 while in manual mode
float targetCurrentA = DEFAULT_MANUAL_CURRENT_A;

// Big-digit overlay: rotating the encoder (to scroll a capacity preset
// or adjust manual current) or long-pressing (to switch mode) shows a
// full-screen readout instead of the normal status screen, until
// bigOverlayUntilMs passes -- see updateDisplay()/drawBigOverlay(). The
// underlying value is applied live as you turn it; there's no separate
// confirm step. overlaySettingsDirty defers the NVS write until the
// overlay actually dismisses, so spinning the encoder fast doesn't
// hammer flash with a write per detent.
unsigned long bigOverlayUntilMs = 0; // 0 or in the past = not showing
bool overlaySettingsDirty = false;

float busVoltage = 0.0f;
float busCurrent = 0.0f; // amps, always >= 0 for this design (one-directional charging)
float ahDelivered = 0.0f;
float initialSocFraction = 0.0f; // estimated from OCV before this charge session started
float socFraction = 0.0f;        // current estimate, 0..1

unsigned long chargeStartMs = 0;
unsigned long absorptionStartMs = 0;
unsigned long belowTerminationSinceMs = 0; // 0 = not currently below threshold
unsigned long overCurrentFaultSinceMs = 0; // 0 = not currently over

uint16_t gateDuty = 0;          // current commanded duty, 0..GATE_PWM_MAX_DUTY
bool softStarting = false;
unsigned long softStartBeginMs = 0;

float piIntegral = 0.0f; // shared integral accumulator, reset on every stage/charge transition

unsigned long lastControlLoopMs = 0;
unsigned long lastDisplayMs = 0;

// Encoder state (quadrature decode via ISR on both A and B pins, using a
// full-state transition table -- see onEncoderChange() for why)
volatile int32_t encoderRawCount = 0;
volatile uint8_t encoderPrevState = 0;
int32_t encoderLastConsumedCount = 0;

// Button state
bool buttonLastRaw = false;      // true = pressed (active low, so this is !digitalRead)
bool buttonStable = false;
unsigned long buttonLastChangeMs = 0;
bool buttonLongPressFired = false;
unsigned long buttonPressStartMs = 0;

// Buzzer state (non-blocking tone sequencer -- see updateBuzzer())
const uint16_t* activeBeepFreqs = nullptr;
const uint16_t* activeBeepDurs  = nullptr;
uint8_t activeBeepLen   = 0;
uint8_t activeBeepIndex = 0;
unsigned long beepStepStartMs = 0;
bool beeping = false;

// ------------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------------
String resetReasonString();
void startCharging();
void stopCharging(ChargeState endState, const String &reason);
void controlLoop();
void updateDisplay();
void drawStrFit(int x, int y, int maxWidth, const char* text);
void drawBatteryIcon(int x, int y, int w, int h, float fraction);
void drawBigOverlay();
void handleEncoderAndButton();
void applyGateDuty(uint16_t duty);
float estimateSocFromOcv(float v);
void loadPersistedSettings();
void savePersistedSettings();
void setStatusLed(uint8_t r, uint8_t g, uint8_t b);
void ina219WriteCalibration(uint16_t calValue);
int16_t ina219ReadRawCurrent();
float readIna219CurrentA();
void startBeepPattern(const uint16_t* freqs, const uint16_t* durs, uint8_t len);
void updateBuzzer();
float readSupplyVoltage();
float readBatteryVoltage();

// ------------------------------------------------------------------
// Helpers
// ------------------------------------------------------------------
String resetReasonString() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON:   return "Power on";
    case ESP_RST_EXT:       return "External pin";
    case ESP_RST_SW:        return "Software reset";
    case ESP_RST_PANIC:     return "Panic/exception";
    case ESP_RST_INT_WDT:   return "Interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "Task watchdog";
    case ESP_RST_WDT:       return "Other watchdog";
    case ESP_RST_DEEPSLEEP: return "Deep sleep wake";
    case ESP_RST_BROWNOUT:  return "Brownout";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "Unknown";
  }
}

// Rough resting-voltage -> state-of-charge lookup for a 12V flooded/AGM
// lead-acid battery. Only meaningful when the battery has been resting
// (no load, not charging) -- called once, right before a charge session
// starts, using the INA219 reading taken with the MOSFET still off.
float estimateSocFromOcv(float v) {
  struct Point { float v; float soc; };
  static const Point table[] = {
    {11.31f, 0.00f}, {11.58f, 0.10f}, {11.75f, 0.20f}, {11.90f, 0.30f},
    {12.06f, 0.40f}, {12.20f, 0.50f}, {12.32f, 0.60f}, {12.42f, 0.70f},
    {12.50f, 0.80f}, {12.60f, 0.90f}, {12.70f, 1.00f},
  };
  static const int n = sizeof(table) / sizeof(table[0]);
  if (v <= table[0].v) return table[0].soc;
  if (v >= table[n - 1].v) return table[n - 1].soc;
  for (int i = 0; i < n - 1; i++) {
    if (v >= table[i].v && v <= table[i + 1].v) {
      float frac = (v - table[i].v) / (table[i + 1].v - table[i].v);
      return table[i].soc + frac * (table[i + 1].soc - table[i].soc);
    }
  }
  return 0.5f;
}

void setStatusLed(uint8_t r, uint8_t g, uint8_t b) {
#if __has_include(<Adafruit_NeoPixel.h>)
  if (!STATUS_LED_ENABLED) return;
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
#endif
}

void updateStatusLedForState() {
  switch (chargeState) {
    case STATE_IDLE:       setStatusLed(0, 0, STATUS_LED_BRIGHTNESS); break;             // blue
    case STATE_BULK:       setStatusLed(STATUS_LED_BRIGHTNESS, STATUS_LED_BRIGHTNESS, 0); break; // yellow
    case STATE_ABSORPTION: setStatusLed(STATUS_LED_BRIGHTNESS, STATUS_LED_BRIGHTNESS / 2, 0); break; // orange
    case STATE_DONE:       setStatusLed(0, STATUS_LED_BRIGHTNESS, 0); break;             // green
    case STATE_FAULT:      setStatusLed(STATUS_LED_BRIGHTNESS, 0, 0); break;             // red
  }
}

#define BEEP_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

// ------------------------------------------------------------------
// Piezo buzzer -- non-blocking tone sequencer. startBeepPattern() kicks
// off a pattern (parallel frequency/duration arrays, see config.h.example
// for the BUZZER_* patterns); updateBuzzer(), called every loop()
// iteration, steps through it by elapsed time without ever blocking the
// control loop or encoder handling. A frequency of 0 is a silent gap.
// ------------------------------------------------------------------
void startBeepPattern(const uint16_t* freqs, const uint16_t* durs, uint8_t len) {
  if (!BUZZER_ENABLED || len == 0) return;
  activeBeepFreqs = freqs;
  activeBeepDurs = durs;
  activeBeepLen = len;
  activeBeepIndex = 0;
  beepStepStartMs = millis();
  beeping = true;
  if (freqs[0] > 0) tone(BUZZER_PIN, freqs[0]); else noTone(BUZZER_PIN);
}

void updateBuzzer() {
  if (!beeping) return;
  if (millis() - beepStepStartMs < activeBeepDurs[activeBeepIndex]) return;
  activeBeepIndex++;
  if (activeBeepIndex >= activeBeepLen) {
    noTone(BUZZER_PIN);
    beeping = false;
    return;
  }
  beepStepStartMs = millis();
  uint16_t f = activeBeepFreqs[activeBeepIndex];
  if (f > 0) tone(BUZZER_PIN, f); else noTone(BUZZER_PIN);
}

// ------------------------------------------------------------------
// INA219 custom calibration -- see config.h.example for how
// INA219_CALIBRATION / INA219_CURRENT_LSB_A were derived for the 0.01 ohm
// shunt. The Adafruit_INA219 library only ships fixed presets
// (setCalibration_32V2A/32V1A/16V400mA) tied to a 0.1 ohm shunt, so
// getCurrent_mA() would silently use the wrong scale factor for this
// shunt -- instead we write the calibration register ourselves and read
// the raw current register directly, scaling by our own Current_LSB.
// INA219_REG_CALIBRATION/INA219_REG_CURRENT are #defined by
// Adafruit_INA219.h itself (as 0x05/0x04) -- used directly below rather
// than redeclared, since redeclaring the same names as our own constants
// collides with the library's #define at the preprocessor level.
// ------------------------------------------------------------------
void ina219WriteCalibration(uint16_t calValue) {
  Wire.beginTransmission(INA219_I2C_ADDR);
  Wire.write(INA219_REG_CALIBRATION);
  Wire.write((uint8_t)(calValue >> 8));
  Wire.write((uint8_t)(calValue & 0xFF));
  Wire.endTransmission();
}

int16_t ina219ReadRawCurrent() {
  Wire.beginTransmission(INA219_I2C_ADDR);
  Wire.write(INA219_REG_CURRENT);
  if (Wire.endTransmission(false) != 0) return 0; // repeated start; bus error -> report 0A rather than garbage
  if (Wire.requestFrom((uint8_t)INA219_I2C_ADDR, (uint8_t)2) != 2) return 0;
  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  return (int16_t)raw;
}

float readIna219CurrentA() {
  return ina219ReadRawCurrent() * INA219_CURRENT_LSB_A;
}

// ------------------------------------------------------------------
// Supply-rail voltage sense (separate from the INA219) -- see the long
// comment on SUPPLY_VOLTAGE_DIVIDER_PIN in config.h.example for why this
// exists: the MOSFET bank's low-side placement means the INA219's own
// bus-voltage reading is the battery-negative return rail, not battery
// voltage. True battery voltage is computed as
// (this reading) - (INA219 bus reading) in controlLoop()/startCharging().
// analogReadMilliVolts() uses the ESP32's factory ADC calibration --
// meaningfully more accurate than assuming a fixed 3.3V/4095 scale.
// ------------------------------------------------------------------
float readSupplyVoltage() {
  uint32_t sumMv = 0;
  for (uint8_t i = 0; i < SUPPLY_ADC_SAMPLES; i++) {
    sumMv += analogReadMilliVolts(SUPPLY_VOLTAGE_DIVIDER_PIN);
  }
  float avgV = (sumMv / (float)SUPPLY_ADC_SAMPLES) / 1000.0f;
  return avgV * SUPPLY_DIVIDER_RATIO;
}

// True battery terminal voltage -- see readSupplyVoltage()'s comment.
float readBatteryVoltage() {
  float supplyV = readSupplyVoltage();
  float returnRailV = ina219.getBusVoltage_V();
  return supplyV - returnRailV;
}

// ------------------------------------------------------------------
// Persistence (NVS) -- only the last manual/easy-mode selection.
// Charging itself never resumes automatically after a reboot.
// ------------------------------------------------------------------
void loadPersistedSettings() {
  prefs.begin(DEVICE_ID, /*readOnly=*/false);
  useEasyMode = prefs.getBool("easyMode", false);
  capacityPresetIndex = prefs.getUChar("capIdx", DEFAULT_CAPACITY_PRESET_INDEX);
  if (capacityPresetIndex >= CAPACITY_PRESETS_COUNT) capacityPresetIndex = DEFAULT_CAPACITY_PRESET_INDEX;
  targetCurrentA = prefs.getFloat("targetA", DEFAULT_MANUAL_CURRENT_A);
  prefs.end();

  if (useEasyMode) {
    selectedCapacityAh = CAPACITY_PRESETS_AH[capacityPresetIndex];
    targetCurrentA = min(MAX_CHARGE_CURRENT_A, selectedCapacityAh * BULK_CURRENT_FRACTION_OF_CAPACITY);
  } else {
    selectedCapacityAh = 0;
  }
  targetCurrentA = constrain(targetCurrentA, MANUAL_CURRENT_MIN_A, MAX_CHARGE_CURRENT_A);
}

void savePersistedSettings() {
  prefs.begin(DEVICE_ID, /*readOnly=*/false);
  prefs.putBool("easyMode", useEasyMode);
  prefs.putUChar("capIdx", capacityPresetIndex);
  prefs.putFloat("targetA", targetCurrentA);
  prefs.end();
}

// ------------------------------------------------------------------
// MOSFET gate drive
// ------------------------------------------------------------------
void applyGateDuty(uint16_t duty) {
  duty = constrain(duty, (uint16_t)0, GATE_PWM_MAX_DUTY);
  gateDuty = duty;
  ledcWrite(MOSFET_GATE_PIN, duty);
}

// ------------------------------------------------------------------
// Charge control
// ------------------------------------------------------------------
void startCharging() {
  if (chargeState == STATE_BULK || chargeState == STATE_ABSORPTION) return;

  // Take one reading with the MOSFET still off to sanity-check that a
  // battery is actually connected, and to seed the SoC estimate from its
  // resting voltage.
  float ocv = readBatteryVoltage();
  if (ocv < NO_BATTERY_VOLTAGE_THRESHOLD) {
    faultReason = "No battery detected";
    chargeState = STATE_FAULT;
    updateStatusLedForState();
    startBeepPattern(BUZZER_FAULT_FREQ_HZ, BUZZER_FAULT_DUR_MS, BEEP_LEN(BUZZER_FAULT_FREQ_HZ));
    Serial.printf("[charge] refusing to start: OCV=%.2fV below threshold\n", ocv);
    return;
  }
  if (ocv >= OVER_VOLTAGE_CUTOFF) {
    faultReason = "Voltage already too high";
    chargeState = STATE_FAULT;
    updateStatusLedForState();
    startBeepPattern(BUZZER_FAULT_FREQ_HZ, BUZZER_FAULT_DUR_MS, BEEP_LEN(BUZZER_FAULT_FREQ_HZ));
    Serial.printf("[charge] refusing to start: OCV=%.2fV at/above cutoff\n", ocv);
    return;
  }

  initialSocFraction = estimateSocFromOcv(ocv);
  socFraction = initialSocFraction;
  ahDelivered = 0.0f;
  piIntegral = 0.0f;
  belowTerminationSinceMs = 0;
  overCurrentFaultSinceMs = 0;
  chargeStartMs = millis();
  chargeState = STATE_BULK;
  faultReason = "";

  softStarting = true;
  softStartBeginMs = millis();
  applyGateDuty(0);

  Serial.printf("[charge] START target=%.2fA ocv=%.2fV initialSoC=%.0f%%\n",
                targetCurrentA, ocv, initialSocFraction * 100.0f);
  updateStatusLedForState();
  startBeepPattern(BUZZER_START_FREQ_HZ, BUZZER_START_DUR_MS, BEEP_LEN(BUZZER_START_FREQ_HZ));
}

void stopCharging(ChargeState endState, const String &reason) {
  applyGateDuty(0);
  softStarting = false;
  chargeState = endState;
  faultReason = reason;
  Serial.printf("[charge] STOP -> %s (%s)\n", chargeStateName(endState), reason.c_str());
  updateStatusLedForState();
  if (endState == STATE_DONE) {
    startBeepPattern(BUZZER_DONE_FREQ_HZ, BUZZER_DONE_DUR_MS, BEEP_LEN(BUZZER_DONE_FREQ_HZ));
  } else if (endState == STATE_FAULT) {
    startBeepPattern(BUZZER_FAULT_FREQ_HZ, BUZZER_FAULT_DUR_MS, BEEP_LEN(BUZZER_FAULT_FREQ_HZ));
  } else {
    startBeepPattern(BUZZER_STOP_FREQ_HZ, BUZZER_STOP_DUR_MS, BEEP_LEN(BUZZER_STOP_FREQ_HZ));
  }
}

// Called every CONTROL_LOOP_INTERVAL_MS. Reads the sensor, runs safety
// checks, advances the state machine, and (while actively charging)
// updates the gate PWM duty via a simple PI loop.
void controlLoop() {
  unsigned long now = millis();
  float dtSec = (now - lastControlLoopMs) / 1000.0f;
  lastControlLoopMs = now;
  if (dtSec <= 0 || dtSec > 5.0f) dtSec = CONTROL_LOOP_INTERVAL_MS / 1000.0f; // guard first call / overflow

  busVoltage = readBatteryVoltage();
  busCurrent = max(0.0f, readIna219CurrentA()); // this design only ever sources current one way

  if (chargeState != STATE_BULK && chargeState != STATE_ABSORPTION) return;

  // ---- Safety nets, independent of the control loop below ----
  if (busVoltage >= OVER_VOLTAGE_CUTOFF) {
    stopCharging(STATE_FAULT, "Over-voltage cutoff");
    return;
  }
  if (now - chargeStartMs >= MAX_CHARGE_DURATION_MS) {
    stopCharging(STATE_FAULT, "Max charge duration exceeded");
    return;
  }
  if (busCurrent >= MAX_CHARGE_CURRENT_A * OVER_CURRENT_FAULT_FACTOR) {
    if (overCurrentFaultSinceMs == 0) overCurrentFaultSinceMs = now;
    if (now - overCurrentFaultSinceMs >= OVER_CURRENT_FAULT_DWELL_MS) {
      stopCharging(STATE_FAULT, "Over-current fault");
      return;
    }
  } else {
    overCurrentFaultSinceMs = 0;
  }

  // ---- Coulomb counting / SoC estimate ----
  ahDelivered += busCurrent * (dtSec / 3600.0f);
  float capacityForSoc = selectedCapacityAh > 0 ? selectedCapacityAh : (targetCurrentA / BULK_CURRENT_FRACTION_OF_CAPACITY);
  if (capacityForSoc > 0) {
    socFraction = constrain(initialSocFraction + (ahDelivered * CHARGE_EFFICIENCY) / capacityForSoc, 0.0f, 1.0f);
  }

  // ---- Soft-start ramp: ease the duty up over SOFT_START_RAMP_MS instead
  // of jumping straight to a PI-computed value, to avoid an inrush step
  // change on the MOSFET/battery every time charging (re)starts. ----
  uint16_t maxDutyThisTick = GATE_PWM_MAX_DUTY;
  if (softStarting) {
    unsigned long elapsed = now - softStartBeginMs;
    if (elapsed >= SOFT_START_RAMP_MS) {
      softStarting = false;
      maxDutyThisTick = GATE_PWM_MAX_DUTY;
    } else {
      maxDutyThisTick = (uint32_t)GATE_PWM_MAX_DUTY * elapsed / SOFT_START_RAMP_MS;
    }
  }

  // ---- State machine + PI control ----
  if (chargeState == STATE_BULK) {
    if (busVoltage >= BULK_TARGET_VOLTAGE) {
      chargeState = STATE_ABSORPTION;
      absorptionStartMs = now;
      piIntegral = 0.0f;
      belowTerminationSinceMs = 0;
      Serial.println("[charge] BULK -> ABSORPTION");
      updateStatusLedForState();
    } else {
      float error = targetCurrentA - busCurrent;
      piIntegral = constrain(piIntegral + error * dtSec, -PI_INTEGRAL_CLAMP, PI_INTEGRAL_CLAMP);
      float duty = BULK_KP * error + BULK_KI * piIntegral;
      applyGateDuty((uint16_t)constrain(duty, 0.0f, (float)maxDutyThisTick));
    }
  } else if (chargeState == STATE_ABSORPTION) {
    float error = ABSORPTION_VOLTAGE - busVoltage;
    piIntegral = constrain(piIntegral + error * dtSec, -PI_INTEGRAL_CLAMP, PI_INTEGRAL_CLAMP);
    float duty = ABS_KP * error + ABS_KI * piIntegral;
    applyGateDuty((uint16_t)constrain(duty, 0.0f, (float)maxDutyThisTick));

    float capacityForTerm = selectedCapacityAh > 0 ? selectedCapacityAh : (targetCurrentA / BULK_CURRENT_FRACTION_OF_CAPACITY);
    float terminationCurrent = capacityForTerm * TERMINATION_CURRENT_FRACTION;
    bool longEnoughInAbsorption = (now - absorptionStartMs) >= MIN_ABSORPTION_MS;

    if (longEnoughInAbsorption && busCurrent <= terminationCurrent) {
      if (belowTerminationSinceMs == 0) belowTerminationSinceMs = now;
      if (now - belowTerminationSinceMs >= MIN_TERMINATION_DWELL_MS) {
        socFraction = 1.0f;
        stopCharging(STATE_DONE, "Charge complete");
      }
    } else {
      belowTerminationSinceMs = 0;
    }
  }

  // Throttled tuning log -- watch this in the Serial Monitor while
  // adjusting BULK_KP/BULK_KI/ABS_KP/ABS_KI against your actual RC
  // filter/MOSFET: duty should move smoothly toward a steady value, not
  // oscillate or slam into 0/max repeatedly.
  static unsigned long lastControlLogMs = 0;
  if (now - lastControlLogMs >= 1000) {
    lastControlLogMs = now;
    Serial.printf("[ctrl] state=%s V=%.2f I=%.2f target=%.2fA duty=%u/%u soc=%.0f%%\n",
                  chargeStateName(chargeState), busVoltage, busCurrent, targetCurrentA,
                  gateDuty, GATE_PWM_MAX_DUTY, socFraction * 100.0f);
  }
}

// ------------------------------------------------------------------
// Encoder (interrupt-driven quadrature decode) + button (polled)
//
// A naive "just compare A and B on every A edge" decoder (the previous
// version of this function) has no way to tell a real detent click
// apart from mechanical contact bounce -- every bounce toggles the pin
// and gets counted as a real step, which is exactly the "bouncing and
// jumping" symptom. This version reads BOTH A and B on every edge of
// EITHER pin, and looks up the (previous state, new state) pair in a
// full quadrature state table: legitimate single-step transitions along
// the Gray-code sequence (00-01-11-10-00, or its reverse) return +-1,
// and anything else -- no change, or an impossible two-bit jump, which
// is what most bounce looks like -- returns 0 and is ignored. No timer-
// based debounce needed; the table does it structurally.
// ------------------------------------------------------------------
void IRAM_ATTR onEncoderChange() {
  static const int8_t QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };
  uint8_t a = digitalRead(ENCODER_A_PIN);
  uint8_t b = digitalRead(ENCODER_B_PIN);
  uint8_t currState = (a << 1) | b;
  uint8_t index = (encoderPrevState << 2) | currState;
  encoderRawCount += QUAD_TABLE[index];
  encoderPrevState = currState;
}

// Consumes accumulated encoder ticks and any button edges, and updates
// charge/UI state. Called every loop() iteration -- cheap when nothing
// changed.
//
// Interaction model: no separate "picker" screen or confirm step.
// Rotating while idle adjusts the live value directly -- the capacity
// preset if in easy mode, the manual target current otherwise -- and
// pops up a full-screen readout (drawBigOverlay()) that auto-dismisses
// BIG_OVERLAY_SCROLL_MS after the last tick. Long-pressing instantly
// toggles between easy and manual mode and shows the same overlay for
// the shorter BIG_OVERLAY_MODE_SWITCH_MS. The click button's role is
// unchanged by any of this -- start/stop/acknowledge -- and works the
// same whether or not the overlay happens to be showing.
void handleEncoderAndButton() {
  unsigned long now = millis();

  // A deferred settings save from the last rotation lands once the
  // overlay it triggered actually dismisses, rather than on every
  // single detent while the user is still spinning the encoder.
  if (overlaySettingsDirty && now >= bigOverlayUntilMs) {
    savePersistedSettings();
    overlaySettingsDirty = false;
  }

  // ---- Encoder rotation ----
  noInterrupts();
  int32_t raw = encoderRawCount;
  interrupts();
  int32_t rawDelta = raw - encoderLastConsumedCount;
  int32_t rawDetents = rawDelta / ENCODER_EDGES_PER_DETENT;
  if (rawDetents != 0) {
    encoderLastConsumedCount += rawDetents * ENCODER_EDGES_PER_DETENT;
    int32_t detents = ENCODER_REVERSED ? -rawDetents : rawDetents;

    if (chargeState == STATE_IDLE) {
      if (useEasyMode) {
        int newIdx = (int)capacityPresetIndex + detents;
        capacityPresetIndex = (uint8_t)constrain(newIdx, 0, (int)CAPACITY_PRESETS_COUNT - 1);
        selectedCapacityAh = CAPACITY_PRESETS_AH[capacityPresetIndex];
        targetCurrentA = constrain(selectedCapacityAh * BULK_CURRENT_FRACTION_OF_CAPACITY,
                                    MANUAL_CURRENT_MIN_A, MAX_CHARGE_CURRENT_A);
      } else {
        float newTarget = targetCurrentA + detents * MANUAL_CURRENT_STEP_A;
        targetCurrentA = constrain(newTarget, MANUAL_CURRENT_MIN_A, MAX_CHARGE_CURRENT_A);
      }
      overlaySettingsDirty = true;
      bigOverlayUntilMs = now + BIG_OVERLAY_SCROLL_MS;
    }
  }

  // ---- Button (active low) with debounce + long-press detection ----
  bool rawPressed = (digitalRead(ENCODER_SW_PIN) == LOW);
  if (rawPressed != buttonLastRaw) {
    buttonLastRaw = rawPressed;
    buttonLastChangeMs = now;
  }
  if ((now - buttonLastChangeMs) >= BUTTON_DEBOUNCE_MS && rawPressed != buttonStable) {
    buttonStable = rawPressed;
    if (buttonStable) {
      // press started
      buttonPressStartMs = now;
      buttonLongPressFired = false;
    } else {
      // release -- if a long press already fired, this release does nothing more
      if (!buttonLongPressFired) {
        // ---- short click ----
        if (chargeState == STATE_IDLE) {
          startCharging();
        } else if (chargeState == STATE_BULK || chargeState == STATE_ABSORPTION) {
          stopCharging(STATE_IDLE, "Stopped by user");
        } else { // DONE or FAULT -- acknowledge and return to idle
          chargeState = STATE_IDLE;
          faultReason = "";
          updateStatusLedForState();
        }
      }
    }
  }

  // long-press fires once, while still held, without waiting for release
  if (buttonStable && !buttonLongPressFired && (now - buttonPressStartMs) >= LONG_PRESS_MS) {
    buttonLongPressFired = true;
    if (chargeState == STATE_IDLE) {
      useEasyMode = !useEasyMode;
      if (useEasyMode) {
        selectedCapacityAh = CAPACITY_PRESETS_AH[capacityPresetIndex];
        targetCurrentA = constrain(selectedCapacityAh * BULK_CURRENT_FRACTION_OF_CAPACITY,
                                    MANUAL_CURRENT_MIN_A, MAX_CHARGE_CURRENT_A);
      } else {
        selectedCapacityAh = 0;
      }
      savePersistedSettings();
      bigOverlayUntilMs = now + BIG_OVERLAY_MODE_SWITCH_MS;
      Serial.printf("[ui] switched to %s mode\n", useEasyMode ? "easy" : "manual");
    }
  }
}

// ------------------------------------------------------------------
// Display
// ------------------------------------------------------------------

// Draws text truncated with "..." if it would exceed maxWidth pixels in
// the currently selected font. This is a general safety net, not a
// one-off string fix: a fixed-length assumption is exactly what let
// "(click=start)" and several fault-reason strings silently run off the
// right edge of a 128px display before -- this makes that class of bug
// structurally impossible instead of relying on hand-counting characters
// for every string, forever, including ones added later.
void drawStrFit(int x, int y, int maxWidth, const char* text) {
  if (u8g2.getStrWidth(text) <= maxWidth) {
    u8g2.drawStr(x, y, text);
    return;
  }
  char buf[48];
  size_t len = strlen(text);
  if (len >= sizeof(buf)) len = sizeof(buf) - 1;
  memcpy(buf, text, len);
  buf[len] = '\0';
  while (len > 0) {
    len--;
    buf[len] = '\0';
    String withDots = String(buf) + "...";
    if (u8g2.getStrWidth(withDots.c_str()) <= maxWidth) {
      u8g2.drawStr(x, y, withDots.c_str());
      return;
    }
  }
  u8g2.drawStr(x, y, "...");
}

// Simple hand-drawn battery glyph (body + terminal nub + a fill bar
// proportional to fraction) rather than a generic progress bar -- reads
// at a glance as "this is the battery's charge level" the way a plain
// rectangle doesn't. Occupies a (w+3)xh px footprint at (x, y); w is the
// body width (excluding the terminal nub).
void drawBatteryIcon(int x, int y, int w, int h, float fraction) {
  fraction = constrain(fraction, 0.0f, 1.0f);
  u8g2.drawFrame(x, y, w, h);
  u8g2.drawBox(x + w, y + (h - 5) / 2, 3, 5);
  int fillW = (int)((w - 4) * fraction);
  if (fillW > 0) u8g2.drawBox(x + 2, y + 2, fillW, h - 4);
}

// Full-screen readout shown while scrolling a value or right after a
// mode switch -- see the interaction-model comment on
// handleEncoderAndButton(). Tries the biggest of three font sizes that
// still fits the actual string so a short value ("3.5A") gets to fill
// the screen while the longest possible one ("100Ah") never clips; the
// smallest of the three (logisoso16, already used elsewhere on this
// display) is known to fit any string this function ever produces, so
// it's a safe last resort.
void drawBigOverlay() {
  char buf[16];
  const char* label;
  if (useEasyMode) {
    snprintf(buf, sizeof(buf), "%uAh", selectedCapacityAh);
    label = "Battery capacity";
  } else {
    snprintf(buf, sizeof(buf), "%.1fA", targetCurrentA);
    label = "Manual current";
  }

  u8g2.setFont(u8g2_font_6x10_tf);
  drawStrFit(0, 9, 128, label);
  u8g2.drawHLine(0, 12, 128);

  const uint8_t* bigFonts[] = {u8g2_font_logisoso32_tf, u8g2_font_logisoso24_tf, u8g2_font_logisoso16_tf};
  const int baselineY[] = {58, 50, 40};
  for (uint8_t i = 0; i < 3; i++) {
    u8g2.setFont(bigFonts[i]);
    int w = u8g2.getStrWidth(buf);
    if (w <= 124 || i == 2) {
      u8g2.drawStr((128 - w) / 2, baselineY[i], buf);
      break;
    }
  }
}

void updateDisplay() {
  u8g2.clearBuffer();

  if (chargeState == STATE_IDLE && millis() < bigOverlayUntilMs) {
    drawBigOverlay();
    u8g2.sendBuffer();
    return;
  }

  // Header: state name (bold) on the left, capacity/mode badge on the right
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 11, chargeStateName(chargeState));
  char badge[12];
  if (useEasyMode) snprintf(badge, sizeof(badge), "%uAh", selectedCapacityAh);
  else snprintf(badge, sizeof(badge), "MANUAL");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(128 - u8g2.getStrWidth(badge), 10, badge);
  u8g2.drawHLine(0, 13, 128);

  // Battery icon (~4x its original length) + SoC%, with a small dot
  // while actively charging
  drawBatteryIcon(0, 17, 86, 11, socFraction);
  char socStr[8];
  snprintf(socStr, sizeof(socStr), "%.0f%%", socFraction * 100.0f);
  u8g2.drawStr(94, 27, socStr);
  if (chargeState == STATE_BULK || chargeState == STATE_ABSORPTION) {
    u8g2.drawDisc(123, 23, 2);
  }

  // Big voltage / current readout, side by side
  char line[16];
  u8g2.setFont(u8g2_font_10x20_tf);
  snprintf(line, sizeof(line), "%.2fV", busVoltage);
  u8g2.drawStr(0, 48, line);
  snprintf(line, sizeof(line), "%.1fA", busCurrent);
  u8g2.drawStr(68, 48, line);

  // Bottom context line -- always width-checked, never assumed to fit
  u8g2.setFont(u8g2_font_6x10_tf);
  if (chargeState == STATE_FAULT) {
    drawStrFit(0, 62, 128, faultReason.c_str());
  } else if (chargeState == STATE_DONE) {
    drawStrFit(0, 62, 128, "Charged -- click to reset");
  } else if (chargeState == STATE_ABSORPTION) {
    drawStrFit(0, 62, 128, "Topping off...");
  } else if (chargeState == STATE_BULK) {
    float remainingAh = max(0.0f, (1.0f - socFraction)) *
        (selectedCapacityAh > 0 ? selectedCapacityAh : (targetCurrentA / BULK_CURRENT_FRACTION_OF_CAPACITY));
    float etaHours = (busCurrent > 0.05f) ? (remainingAh / busCurrent) : 0.0f;
    int hh = (int)etaHours;
    int mm = (int)((etaHours - hh) * 60);
    char etaLine[24];
    snprintf(etaLine, sizeof(etaLine), "ETA ~%dh%02dm to absorb", hh, mm);
    drawStrFit(0, 62, 128, etaLine);
  } else { // IDLE
    char idleLine[24];
    snprintf(idleLine, sizeof(idleLine), "%.1fA - click to start", targetCurrentA);
    drawStrFit(0, 62, 128, idleLine);
  }

  u8g2.sendBuffer();
}

// ------------------------------------------------------------------
// Setup / loop
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[Car Battery Charger] booting, reset reason: " + resetReasonString());
  Serial.printf("[fw] version %s\n", FIRMWARE_VERSION);

  // Gate pin: pulled low immediately, before ledcAttach, so there's no
  // window where it floats high and turns the MOSFET on.
  pinMode(MOSFET_GATE_PIN, OUTPUT);
  digitalWrite(MOSFET_GATE_PIN, LOW);
  bool ledcOk = ledcAttach(MOSFET_GATE_PIN, GATE_PWM_FREQ_HZ, GATE_PWM_RESOLUTION);
  Serial.printf("[gate] ledcAttach(pin=%d, freq=%uHz, res=%ubit) = %s\n",
                MOSFET_GATE_PIN, (unsigned)GATE_PWM_FREQ_HZ, (unsigned)GATE_PWM_RESOLUTION,
                ledcOk ? "ok" : "FAILED");
  applyGateDuty(0);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  encoderPrevState = (digitalRead(ENCODER_A_PIN) << 1) | digitalRead(ENCODER_B_PIN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), onEncoderChange, CHANGE);

#if __has_include(<Adafruit_NeoPixel.h>)
  if (STATUS_LED_ENABLED) {
    statusLed.begin();
    statusLed.show();
    Serial.println("[led] status LED initialized on GPIO" + String(STATUS_LED_PIN));
  }
#else
  // If this prints, the status LED can never work no matter what else is
  // right -- __has_include() compiles the entire NeoPixel code path out
  // when the library isn't installed, silently, with no other symptom.
  Serial.println("[led] Adafruit_NeoPixel library NOT installed -- status LED will not work. "
                  "Install \"Adafruit NeoPixel\" via Library Manager and re-flash.");
#endif

  if (BUZZER_ENABLED) {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!ina219.begin()) {
    Serial.println("[ina219] FAILED to initialize -- check wiring/address");
  }
  ina219WriteCalibration(INA219_CALIBRATION);
  Serial.printf("[ina219] custom calibration=%u current_lsb=%.6fA/bit (0.01ohm shunt)\n",
                INA219_CALIBRATION, INA219_CURRENT_LSB_A);
  u8g2.begin();

  pinMode(SUPPLY_VOLTAGE_DIVIDER_PIN, INPUT);
  Serial.printf("[supply] measured rail = %.2fV (calibrate SUPPLY_DIVIDER_RATIO against a multimeter)\n",
                readSupplyVoltage());

  loadPersistedSettings();
  chargeState = STATE_IDLE; // never auto-resume charging after a reboot
  updateStatusLedForState();

  lastControlLoopMs = millis();
  Serial.println("[boot] ready");
}

void loop() {
  handleEncoderAndButton();
  updateBuzzer();

  unsigned long now = millis();
  if (now - lastControlLoopMs >= CONTROL_LOOP_INTERVAL_MS) {
    controlLoop();
  }
  if (now - lastDisplayMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayMs = now;
    updateDisplay();
  }
}
