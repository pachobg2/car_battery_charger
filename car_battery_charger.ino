/*
 * Car Battery Charger — ESP32-C3-Zero smart lead-acid charger
 *
 * 3-stage (bulk / absorption / done) charger for flooded/AGM car
 * batteries, built around:
 *   - INA219 (I2C) for bus voltage + charge current sensing, with a
 *     CUSTOM calibration for a 0.01 ohm / 25W shunt resistor (swapped in
 *     place of the breakout's stock 0.1 ohm shunt) for range up to ~10A.
 *     See config.h.example for how INA219_CALIBRATION/INA219_CURRENT_LSB_A
 *     were computed, and the thermal warning on MAX_CHARGE_CURRENT_A.
 *   - SH1106 128x64 OLED (I2C, via U8g2) for status/progress display
 *   - Rotary encoder with push button for local control
 *   - IRL540N MOSFET run in its LINEAR region (PWM through an RC filter
 *     onto the gate) as the current-regulating element -- there is no
 *     inductor/buck stage in this design. See config.h.example for the
 *     gate drive schematic and an important thermal warning: the MOSFET
 *     dissipates real, continuous heat and needs a proper heatsink.
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
 * Local UI: rotate the encoder to set manual charge current, click to
 * start/stop, long-press to enter "easy mode" (pick a capacity preset
 * from config.h's CAPACITY_PRESETS_AH, which auto-computes the bulk
 * current from BULK_CURRENT_FRACTION_OF_CAPACITY).
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

enum UiMode { UI_MAIN, UI_SELECT_CAPACITY };

// ------------------------------------------------------------------
// Globals
// ------------------------------------------------------------------
Preferences prefs;
Adafruit_INA219 ina219(INA219_I2C_ADDR);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /*reset=*/U8X8_PIN_NONE);
#if __has_include(<Adafruit_NeoPixel.h>)
Adafruit_NeoPixel statusLed(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

ChargeState chargeState = STATE_IDLE;
UiMode uiMode = UI_MAIN;
String faultReason = "";

bool useEasyMode = false;
uint8_t capacityPresetIndex = DEFAULT_CAPACITY_PRESET_INDEX;
uint16_t selectedCapacityAh = 0; // 0 while in manual mode
float targetCurrentA = DEFAULT_MANUAL_CURRENT_A;
uint8_t capacitySelectCursor = DEFAULT_CAPACITY_PRESET_INDEX; // scroll position while choosing

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

// Encoder state (quadrature decode via ISR on the A pin)
volatile int32_t encoderRawCount = 0;
int32_t encoderLastConsumedCount = 0;

// Button state
bool buttonLastRaw = false;      // true = pressed (active low, so this is !digitalRead)
bool buttonStable = false;
unsigned long buttonLastChangeMs = 0;
bool buttonLongPressFired = false;
unsigned long buttonPressStartMs = 0;

// ------------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------------
String resetReasonString();
void startCharging();
void stopCharging(ChargeState endState, const String &reason);
void controlLoop();
void updateDisplay();
void handleEncoderAndButton();
void applyGateDuty(uint16_t duty);
float estimateSocFromOcv(float v);
void loadPersistedSettings();
void savePersistedSettings();
void setStatusLed(uint8_t r, uint8_t g, uint8_t b);
void ina219WriteCalibration(uint16_t calValue);
int16_t ina219ReadRawCurrent();
float readIna219CurrentA();

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

// ------------------------------------------------------------------
// INA219 custom calibration -- see config.h.example for how
// INA219_CALIBRATION / INA219_CURRENT_LSB_A were derived for the 0.01 ohm
// shunt. The Adafruit_INA219 library only ships fixed presets
// (setCalibration_32V2A/32V1A/16V400mA) tied to a 0.1 ohm shunt, so
// getCurrent_mA() would silently use the wrong scale factor for this
// shunt -- instead we write the calibration register ourselves and read
// the raw current register directly, scaling by our own Current_LSB.
// ------------------------------------------------------------------
static const uint8_t INA219_REG_CALIBRATION = 0x05;
static const uint8_t INA219_REG_CURRENT     = 0x04;

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
  float ocv = ina219.getBusVoltage_V();
  if (ocv < NO_BATTERY_VOLTAGE_THRESHOLD) {
    faultReason = "No battery detected";
    chargeState = STATE_FAULT;
    updateStatusLedForState();
    Serial.printf("[charge] refusing to start: OCV=%.2fV below threshold\n", ocv);
    return;
  }
  if (ocv >= OVER_VOLTAGE_CUTOFF) {
    faultReason = "Voltage already too high";
    chargeState = STATE_FAULT;
    updateStatusLedForState();
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
}

void stopCharging(ChargeState endState, const String &reason) {
  applyGateDuty(0);
  softStarting = false;
  chargeState = endState;
  faultReason = reason;
  Serial.printf("[charge] STOP -> %s (%s)\n", chargeStateName(endState), reason.c_str());
  updateStatusLedForState();
}

// Called every CONTROL_LOOP_INTERVAL_MS. Reads the sensor, runs safety
// checks, advances the state machine, and (while actively charging)
// updates the gate PWM duty via a simple PI loop.
void controlLoop() {
  unsigned long now = millis();
  float dtSec = (now - lastControlLoopMs) / 1000.0f;
  lastControlLoopMs = now;
  if (dtSec <= 0 || dtSec > 5.0f) dtSec = CONTROL_LOOP_INTERVAL_MS / 1000.0f; // guard first call / overflow

  busVoltage = ina219.getBusVoltage_V();
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
// ------------------------------------------------------------------
void IRAM_ATTR onEncoderAChange() {
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);
  encoderRawCount += (a == b) ? 1 : -1;
}

// Consumes accumulated encoder ticks and any button edges, and updates
// charge/UI state. Called every loop() iteration -- cheap when nothing
// changed.
void handleEncoderAndButton() {
  unsigned long now = millis();

  // ---- Encoder rotation ----
  noInterrupts();
  int32_t raw = encoderRawCount;
  interrupts();
  int32_t rawDelta = raw - encoderLastConsumedCount;
  int32_t detents = rawDelta / ENCODER_EDGES_PER_DETENT;
  if (detents != 0) {
    encoderLastConsumedCount += detents * ENCODER_EDGES_PER_DETENT;

    if (uiMode == UI_SELECT_CAPACITY) {
      int newCursor = (int)capacitySelectCursor + detents;
      capacitySelectCursor = (uint8_t)constrain(newCursor, 0, CAPACITY_PRESETS_COUNT - 1);
    } else if (uiMode == UI_MAIN && chargeState == STATE_IDLE && !useEasyMode) {
      float newTarget = targetCurrentA + detents * MANUAL_CURRENT_STEP_A;
      targetCurrentA = constrain(newTarget, MANUAL_CURRENT_MIN_A, MAX_CHARGE_CURRENT_A);
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
        if (uiMode == UI_SELECT_CAPACITY) {
          capacityPresetIndex = capacitySelectCursor;
          selectedCapacityAh = CAPACITY_PRESETS_AH[capacityPresetIndex];
          useEasyMode = true;
          targetCurrentA = constrain(selectedCapacityAh * BULK_CURRENT_FRACTION_OF_CAPACITY,
                                      MANUAL_CURRENT_MIN_A, MAX_CHARGE_CURRENT_A);
          savePersistedSettings();
          uiMode = UI_MAIN;
          Serial.printf("[ui] capacity confirmed: %uAh -> target %.2fA\n", selectedCapacityAh, targetCurrentA);
        } else { // UI_MAIN
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
  }

  // long-press fires once, while still held, without waiting for release
  if (buttonStable && !buttonLongPressFired && (now - buttonPressStartMs) >= LONG_PRESS_MS) {
    buttonLongPressFired = true;
    if (chargeState == STATE_IDLE) {
      if (uiMode == UI_MAIN) {
        uiMode = UI_SELECT_CAPACITY;
        capacitySelectCursor = capacityPresetIndex;
        Serial.println("[ui] entering easy-mode capacity select");
      } else {
        uiMode = UI_MAIN; // long-press again cancels back out without changes
        Serial.println("[ui] leaving capacity select (cancelled)");
      }
    }
  }
}

// ------------------------------------------------------------------
// Display
// ------------------------------------------------------------------
void drawProgressBar(int x, int y, int w, int h, float fraction) {
  fraction = constrain(fraction, 0.0f, 1.0f);
  u8g2.drawFrame(x, y, w, h);
  int fillW = (int)((w - 2) * fraction);
  if (fillW > 0) u8g2.drawBox(x + 1, y + 1, fillW, h - 2);
}

void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  if (uiMode == UI_SELECT_CAPACITY) {
    u8g2.drawStr(0, 10, "Select battery (Ah):");
    u8g2.setFont(u8g2_font_10x20_tf);
    char buf[16];
    snprintf(buf, sizeof(buf), "%u Ah", CAPACITY_PRESETS_AH[capacitySelectCursor]);
    u8g2.drawStr(10, 35, buf);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 55, "Click=confirm  Hold=cancel");
    u8g2.sendBuffer();
    return;
  }

  // ---- UI_MAIN ----
  u8g2.drawStr(0, 10, chargeStateName(chargeState));
  if (useEasyMode) {
    char cap[16];
    snprintf(cap, sizeof(cap), "%uAh", selectedCapacityAh);
    u8g2.drawStr(90, 10, cap);
  }

  drawProgressBar(0, 14, 128, 10, socFraction);

  char line[32];
  u8g2.setFont(u8g2_font_10x20_tf);
  snprintf(line, sizeof(line), "%.2fV", busVoltage);
  u8g2.drawStr(0, 42, line);
  snprintf(line, sizeof(line), "%.2fA", busCurrent);
  u8g2.drawStr(70, 42, line);

  u8g2.setFont(u8g2_font_6x10_tf);
  if (chargeState == STATE_FAULT) {
    u8g2.drawStr(0, 55, faultReason.c_str());
  } else if (chargeState == STATE_DONE) {
    u8g2.drawStr(0, 55, "Charged - click to reset");
  } else if (chargeState == STATE_ABSORPTION) {
    u8g2.drawStr(0, 55, "Topping off...");
  } else if (chargeState == STATE_BULK) {
    float remainingAh = max(0.0f, (1.0f - socFraction)) *
        (selectedCapacityAh > 0 ? selectedCapacityAh : (targetCurrentA / BULK_CURRENT_FRACTION_OF_CAPACITY));
    float etaHours = (busCurrent > 0.05f) ? (remainingAh / busCurrent) : 0.0f;
    int hh = (int)etaHours;
    int mm = (int)((etaHours - hh) * 60);
    snprintf(line, sizeof(line), "ETA to absorb ~%dh%02dm", hh, mm);
    u8g2.drawStr(0, 55, line);
  } else { // IDLE
    snprintf(line, sizeof(line), "Target: %.1fA  (click=start)", targetCurrentA);
    u8g2.drawStr(0, 55, line);
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
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onEncoderAChange, CHANGE);

#if __has_include(<Adafruit_NeoPixel.h>)
  if (STATUS_LED_ENABLED) {
    statusLed.begin();
    statusLed.show();
  }
#endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!ina219.begin()) {
    Serial.println("[ina219] FAILED to initialize -- check wiring/address");
  }
  ina219WriteCalibration(INA219_CALIBRATION);
  Serial.printf("[ina219] custom calibration=%u current_lsb=%.6fA/bit (0.01ohm shunt)\n",
                INA219_CALIBRATION, INA219_CURRENT_LSB_A);
  u8g2.begin();

  loadPersistedSettings();
  chargeState = STATE_IDLE; // never auto-resume charging after a reboot
  updateStatusLedForState();

  lastControlLoopMs = millis();
  Serial.println("[boot] ready");
}

void loop() {
  handleEncoderAndButton();

  unsigned long now = millis();
  if (now - lastControlLoopMs >= CONTROL_LOOP_INTERVAL_MS) {
    controlLoop();
  }
  if (now - lastDisplayMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayMs = now;
    updateDisplay();
  }
}
