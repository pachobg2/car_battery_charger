/*
 * Car Battery Charger — ESP32-C3-Zero smart lead-acid charger
 *
 * 3-stage (bulk / absorption / done) charger for flooded/AGM car
 * batteries, built around:
 *   - INA219 (I2C) for bus voltage + charge current sensing
 *   - SH1106 128x64 OLED (I2C, via U8g2) for status/progress display
 *   - Rotary encoder with push button for local control
 *   - IRL540N MOSFET run in its LINEAR region (PWM through an RC filter
 *     onto the gate) as the current-regulating element -- there is no
 *     inductor/buck stage in this design. See config.h.example for the
 *     gate drive schematic and an important thermal warning: the MOSFET
 *     dissipates real, continuous heat and needs a proper heatsink.
 *   - espMqttClient + Home Assistant MQTT discovery, matching the rest
 *     of this fleet, so the charger can also be monitored/controlled
 *     remotely (voltage/current/state/SoC sensors, target-current
 *     number, capacity select, start/stop buttons)
 *   - ArduinoOTA for wireless updates after the first USB flash
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
 *   - espMqttClient (bertmelis)
 *   - Adafruit INA219 (adafruit)
 *   - U8g2 (olikraus)
 *   - Adafruit NeoPixel (adafruit) -- only if STATUS_LED_ENABLED
 *   Built-in / come with the ESP32 Arduino core: WiFi, ArduinoOTA,
 *   Preferences, Wire
 *
 * Board package: esp32 by Espressif Systems -- board "ESP32C3 Dev
 * Module". Rename config.h.example -> config.h and fill in your
 * WiFi/MQTT/OTA credentials and hardware pins before building.
 */

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <espMqttClient.h>
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
espMqttClient mqttClient;
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
uint16_t softStartTargetDuty = 0;
unsigned long softStartBeginMs = 0;
bool softStarting = false;

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

// mqtt reconnect / diagnostics
unsigned long lastMqttAttemptMs = 0;
unsigned long mqttBackoffMs     = 1000;
static const unsigned long MQTT_BACKOFF_MAX_MS = 30000;
uint32_t mqttFailCount = 0;
bool     everConnected = false;
unsigned long lastDiagPublishMs = 0;

// wifi reconnect state (non-blocking)
bool          wifiConnectInProgress = false;
unsigned long wifiConnectStartMs    = 0;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;

// ------------------------------------------------------------------
// MQTT / discovery topics
// ------------------------------------------------------------------
String baseTopic               = String("charger/") + DEVICE_ID;
String availabilityTopic       = baseTopic + "/availability";
String stateTopic              = baseTopic + "/state";               // Idle/Bulk/Absorption/Done/Fault
String voltageTopic            = baseTopic + "/voltage/state";
String currentTopic            = baseTopic + "/current/state";
String socTopic                = baseTopic + "/soc/state";
String ahDeliveredTopic        = baseTopic + "/ah_delivered/state";
String faultTopic              = baseTopic + "/fault/state";
String targetCurrentStateTopic = baseTopic + "/target_current/state";
String targetCurrentCmdTopic   = baseTopic + "/target_current/set";
String capacityStateTopic      = baseTopic + "/capacity/state";
String capacityCmdTopic        = baseTopic + "/capacity/set";
String startCmdTopic           = baseTopic + "/start";
String stopCmdTopic            = baseTopic + "/stop";
String wifiSignalTopic         = baseTopic + "/wifi_signal/state";
String resetReasonTopic        = baseTopic + "/reset_reason/state";
String failCountTopic          = baseTopic + "/mqtt_fail_count/state";
String uptimeTopic             = baseTopic + "/uptime/state";

String discoverySensorPrefix   = String("homeassistant/sensor/") + DEVICE_ID + "_";
String discoveryNumberPrefix   = String("homeassistant/number/") + DEVICE_ID + "_";
String discoverySelectPrefix   = String("homeassistant/select/") + DEVICE_ID + "_";
String discoveryButtonPrefix   = String("homeassistant/button/") + DEVICE_ID + "_";

// ------------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------------
void connectWifi();
void maintainWifi();
bool checkedPublish(const String &topic, uint8_t qos, bool retain, const String &payload);
void connectMqtt();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason);
void onMqttMessage(const espMqttClientTypes::MessageProperties& properties,
                    const char* topic, const uint8_t* payload, size_t len,
                    size_t index, size_t total);
void publishDiscovery();
void publishChargeState();
void publishDiagnostics(bool force);
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
  publishChargeState();
}

void stopCharging(ChargeState endState, const String &reason) {
  applyGateDuty(0);
  softStarting = false;
  chargeState = endState;
  faultReason = reason;
  Serial.printf("[charge] STOP -> %s (%s)\n", chargeStateName(endState), reason.c_str());
  updateStatusLedForState();
  publishChargeState();
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
  float rawCurrentMa = ina219.getCurrent_mA();
  busCurrent = max(0.0f, rawCurrentMa / 1000.0f); // this design only ever sources current one way

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
      publishChargeState();
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
            publishChargeState();
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
// MQTT publish helper
// ------------------------------------------------------------------
bool checkedPublish(const String &topic, uint8_t qos, bool retain, const String &payload) {
  uint16_t packetId = mqttClient.publish(topic.c_str(), qos, retain, payload.c_str());
  if (packetId == 0) {
    Serial.printf("[mqtt] publish FAILED topic=%s free_heap=%u\n", topic.c_str(), ESP.getFreeHeap());
  }
  return packetId != 0;
}

void publishChargeState() {
  if (!mqttClient.connected()) return;
  checkedPublish(stateTopic, 1, true, chargeStateName(chargeState));
  checkedPublish(voltageTopic, 0, false, String(busVoltage, 2));
  checkedPublish(currentTopic, 0, false, String(busCurrent, 2));
  checkedPublish(socTopic, 0, false, String(socFraction * 100.0f, 0));
  checkedPublish(ahDeliveredTopic, 0, false, String(ahDelivered, 2));
  checkedPublish(faultTopic, 0, true, faultReason);
  checkedPublish(targetCurrentStateTopic, 0, true, String(targetCurrentA, 2));
  checkedPublish(capacityStateTopic, 0, true, useEasyMode ? String(selectedCapacityAh) : String("manual"));
}

// ------------------------------------------------------------------
// Home Assistant discovery
// ------------------------------------------------------------------
void publishDiscovery() {
  String deviceJson = String("{") +
      "\"identifiers\":[\"" + DEVICE_ID + "\"]," +
      "\"name\":\"" + DEVICE_NAME + "\"," +
      "\"manufacturer\":\"" + MANUFACTURER + "\"," +
      "\"model\":\"" + MODEL + "\"," +
      "\"sw_version\":\"" + FIRMWARE_VERSION + "\"" +
      "}";

  struct SensorDef { const char* key; const char* name; const String topic; const char* unit; const char* deviceClass; const char* stateClass; };
  SensorDef sensors[] = {
    {"state", "Charge State", stateTopic, nullptr, nullptr, nullptr},
    {"voltage", "Battery Voltage", voltageTopic, "V", "voltage", "measurement"},
    {"current", "Charge Current", currentTopic, "A", "current", "measurement"},
    {"soc", "State of Charge", socTopic, "%", "battery", "measurement"},
    {"ah_delivered", "Ah Delivered", ahDeliveredTopic, "Ah", nullptr, "total_increasing"},
    {"fault", "Fault Reason", faultTopic, nullptr, nullptr, nullptr},
    {"wifi_signal", "WiFi Signal", wifiSignalTopic, "dBm", "signal_strength", "measurement"},
    {"reset_reason", "Reset Reason", resetReasonTopic, nullptr, nullptr, nullptr},
    {"mqtt_fail_count", "MQTT Fail Count", failCountTopic, nullptr, nullptr, "total_increasing"},
    {"uptime", "Uptime", uptimeTopic, "s", nullptr, "measurement"},
  };
  for (auto &s : sensors) {
    bool diagnostic = (strcmp(s.key, "wifi_signal") == 0 || strcmp(s.key, "reset_reason") == 0 ||
                        strcmp(s.key, "mqtt_fail_count") == 0 || strcmp(s.key, "uptime") == 0);
    String payload = String("{") +
        "\"name\":\"" + s.name + "\"," +
        "\"unique_id\":\"" + DEVICE_ID + "_" + s.key + "\"," +
        "\"state_topic\":\"" + s.topic + "\",";
    if (s.unit) payload += String("\"unit_of_measurement\":\"") + s.unit + "\",";
    if (s.deviceClass) payload += String("\"device_class\":\"") + s.deviceClass + "\",";
    if (s.stateClass) payload += String("\"state_class\":\"") + s.stateClass + "\",";
    if (diagnostic) payload += "\"entity_category\":\"diagnostic\",";
    payload += "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson + "}";
    checkedPublish(discoverySensorPrefix + s.key + "/config", 1, true, payload);
  }

  // Target current (number entity)
  {
    String payload = String("{") +
        "\"name\":\"Target Current\"," +
        "\"unique_id\":\"" + DEVICE_ID + "_target_current\"," +
        "\"state_topic\":\"" + targetCurrentStateTopic + "\"," +
        "\"command_topic\":\"" + targetCurrentCmdTopic + "\"," +
        "\"unit_of_measurement\":\"A\"," +
        "\"min\":" + String(MANUAL_CURRENT_MIN_A, 1) + "," +
        "\"max\":" + String(MAX_CHARGE_CURRENT_A, 1) + "," +
        "\"step\":" + String(MANUAL_CURRENT_STEP_A, 1) + "," +
        "\"mode\":\"box\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryNumberPrefix + "target_current/config", 1, true, payload);
  }

  // Capacity preset (select entity)
  {
    String options = "[\"manual\"";
    for (uint8_t i = 0; i < CAPACITY_PRESETS_COUNT; i++) {
      options += ",\"" + String(CAPACITY_PRESETS_AH[i]) + "\"";
    }
    options += "]";
    String payload = String("{") +
        "\"name\":\"Battery Capacity\"," +
        "\"unique_id\":\"" + DEVICE_ID + "_capacity\"," +
        "\"state_topic\":\"" + capacityStateTopic + "\"," +
        "\"command_topic\":\"" + capacityCmdTopic + "\"," +
        "\"options\":" + options + "," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoverySelectPrefix + "capacity/config", 1, true, payload);
  }

  // Start / Stop buttons
  {
    String startPayload = String("{") +
        "\"name\":\"Start Charging\"," +
        "\"unique_id\":\"" + DEVICE_ID + "_start\"," +
        "\"command_topic\":\"" + startCmdTopic + "\"," +
        "\"payload_press\":\"START\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryButtonPrefix + "start/config", 1, true, startPayload);

    String stopPayload = String("{") +
        "\"name\":\"Stop Charging\"," +
        "\"unique_id\":\"" + DEVICE_ID + "_stop\"," +
        "\"command_topic\":\"" + stopCmdTopic + "\"," +
        "\"payload_press\":\"STOP\"," +
        "\"availability_topic\":\"" + availabilityTopic + "\"," +
        "\"device\":" + deviceJson +
        "}";
    checkedPublish(discoveryButtonPrefix + "stop/config", 1, true, stopPayload);
  }
}

// ------------------------------------------------------------------
// MQTT callbacks
// ------------------------------------------------------------------
void onMqttConnect(bool sessionPresent) {
  Serial.println("[mqtt] connected");
  mqttBackoffMs = 1000;
  everConnected = true;

  checkedPublish(availabilityTopic, 1, true, "online");

  mqttClient.subscribe(targetCurrentCmdTopic.c_str(), 1);
  mqttClient.subscribe(capacityCmdTopic.c_str(), 1);
  mqttClient.subscribe(startCmdTopic.c_str(), 1);
  mqttClient.subscribe(stopCmdTopic.c_str(), 1);

  publishDiscovery();
  publishChargeState();
  publishDiagnostics(true);
}

void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason) {
  Serial.printf("[mqtt] disconnected, reason: %u\n", static_cast<uint8_t>(reason));
  if (everConnected) mqttFailCount++;
  mqttBackoffMs = min(mqttBackoffMs * 2, MQTT_BACKOFF_MAX_MS);
}

void onMqttMessage(const espMqttClientTypes::MessageProperties& properties,
                    const char* topic, const uint8_t* payload, size_t len,
                    size_t index, size_t total) {
  String topicStr(topic);
  String payloadStr;
  payloadStr.reserve(len);
  for (size_t i = 0; i < len; i++) payloadStr += (char)payload[i];

  Serial.printf("[mqtt] message topic=%s payload=%s\n", topicStr.c_str(), payloadStr.c_str());

  if (topicStr == targetCurrentCmdTopic) {
    if (chargeState != STATE_IDLE) {
      Serial.println("[mqtt] ignoring target_current change while charging (stop first)");
      return;
    }
    float v = payloadStr.toFloat();
    useEasyMode = false;
    selectedCapacityAh = 0;
    targetCurrentA = constrain(v, MANUAL_CURRENT_MIN_A, MAX_CHARGE_CURRENT_A);
    savePersistedSettings();
    publishChargeState();
  } else if (topicStr == capacityCmdTopic) {
    if (chargeState != STATE_IDLE) {
      Serial.println("[mqtt] ignoring capacity change while charging (stop first)");
      return;
    }
    if (payloadStr == "manual") {
      useEasyMode = false;
      selectedCapacityAh = 0;
    } else {
      uint16_t ah = (uint16_t)payloadStr.toInt();
      for (uint8_t i = 0; i < CAPACITY_PRESETS_COUNT; i++) {
        if (CAPACITY_PRESETS_AH[i] == ah) {
          capacityPresetIndex = i;
          useEasyMode = true;
          selectedCapacityAh = ah;
          targetCurrentA = constrain(ah * BULK_CURRENT_FRACTION_OF_CAPACITY, MANUAL_CURRENT_MIN_A, MAX_CHARGE_CURRENT_A);
          break;
        }
      }
    }
    savePersistedSettings();
    publishChargeState();
  } else if (topicStr == startCmdTopic) {
    if (chargeState == STATE_IDLE) startCharging();
  } else if (topicStr == stopCmdTopic) {
    if (chargeState == STATE_BULK || chargeState == STATE_ABSORPTION) {
      stopCharging(STATE_IDLE, "Stopped via MQTT");
    }
  }
}

// ------------------------------------------------------------------
// WiFi -- DHCP only (see plugin_light/smart_switch for why: avoids the
// WiFi.config() race with stale NVS credentials on a reused board)
// ------------------------------------------------------------------
void connectWifi() {
  if (WiFi.status() == WL_CONNECTED || wifiConnectInProgress) return;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(DEVICE_ID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiConnectInProgress = true;
  wifiConnectStartMs = millis();
  Serial.println("[wifi] connecting...");
}

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiConnectInProgress) {
      wifiConnectInProgress = false;
      Serial.printf("[wifi] connected, ip=%s\n", WiFi.localIP().toString().c_str());
    }
    return;
  }
  if (!wifiConnectInProgress) {
    connectWifi();
  } else if (millis() - wifiConnectStartMs > WIFI_CONNECT_TIMEOUT_MS) {
    Serial.println("[wifi] connect attempt timed out, will retry");
    wifiConnectInProgress = false;
  }
}

void connectMqtt() {
  lastMqttAttemptMs = millis();
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.println("[mqtt] connecting...");
  mqttClient.connect();
}

void publishDiagnostics(bool force) {
  if (!mqttClient.connected()) return;
  if (WiFi.status() == WL_CONNECTED) {
    checkedPublish(wifiSignalTopic, 1, true, String(WiFi.RSSI()));
  }
  static bool resetReasonSent = false;
  if (force || !resetReasonSent) {
    checkedPublish(resetReasonTopic, 1, true, resetReasonString());
    resetReasonSent = true;
  }
  checkedPublish(failCountTopic, 1, true, String(mqttFailCount));
  checkedPublish(uptimeTopic, 1, true, String(millis() / 1000));
}

void setupOta() {
  ArduinoOTA.setHostname(DEVICE_ID);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] start");
    applyGateDuty(0); // never leave the charger driving current through an OTA flash
  });
  ArduinoOTA.onEnd([]() { Serial.println("[OTA] done"); });
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("[OTA] error %u\n", (unsigned)error); });
  ArduinoOTA.begin();
}

// ------------------------------------------------------------------
// Setup / loop
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[Car Battery Charger] booting, reset reason: " + resetReasonString());

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
  u8g2.begin();

  loadPersistedSettings();
  chargeState = STATE_IDLE; // never auto-resume charging after a reboot
  updateStatusLedForState();

  connectWifi();
  {
    unsigned long waitStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - waitStart < 5000) delay(100);
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnectInProgress = false;
    Serial.printf("[wifi] connected, ip=%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[wifi] not yet connected at boot, will keep retrying in loop()");
  }

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);
  mqttClient.setClientId(DEVICE_ID);
  mqttClient.setWill(availabilityTopic.c_str(), 1, true, "offline");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  connectMqtt();

  setupOta();

  lastControlLoopMs = millis();
  Serial.println("[boot] ready");
}

void loop() {
  maintainWifi();

  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttAttemptMs >= mqttBackoffMs) connectMqtt();
  }
  mqttClient.loop();
  ArduinoOTA.handle();

  handleEncoderAndButton();

  unsigned long now = millis();
  if (now - lastControlLoopMs >= CONTROL_LOOP_INTERVAL_MS) {
    controlLoop();
  }
  if (now - lastDisplayMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayMs = now;
    updateDisplay();
    if (mqttClient.connected()) publishChargeState();
  }
  if (now - lastDiagPublishMs >= DIAG_INTERVAL_MS) {
    lastDiagPublishMs = now;
    publishDiagnostics(false);
  }
}
