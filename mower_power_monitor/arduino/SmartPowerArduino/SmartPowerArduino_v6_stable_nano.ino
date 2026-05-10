// =====================================================
// SMART BATTERY POWER LATCH / ROS2 SAFE SHUTDOWN
// Arduino Nano / ATmega328P
// Version 6 - stable Nano version, low SRAM, CR/LF serial parser
// =====================================================
//
// Compatible ROS commands, DO NOT change in the ROS node:
//   PI_READY
//   PI_SHUTTING_DOWN <grace_ms>
//   CANCEL_SHUTDOWN
//   STATUS?
//   CUT_NOW
//
// Events/status to ROS:
//   PWRBOOT ...
//   PWRPREV ...
//   PWRSTAT ...
//   PWREVT type=SHUTDOWN_REQUEST reason=... grace_ms=...
//   PWREVT type=POWER_CUT reason=...
//   PWRACK ...
//   PWRERR ...
//
// Button behavior:
//   Short press/release after boot -> graceful Raspberry Pi shutdown request
//   Hold >= 5 seconds             -> immediate emergency power cut
// =====================================================

#include <EEPROM.h>
#include <string.h>
#include <stdlib.h>

// ---------------------------
// PINS
// ---------------------------
const byte holdPin     = 2;
const byte buttonPin   = 3;
const byte batVoltPin  = A0;
const byte currPin     = A1;
const byte ledPin      = 13;

// ---------------------------
// BUTTON WIRING
// ---------------------------
// Current expected wiring, same as your original code:
//   +5V -- button -- D3
//                   |
//                  10k
//                   |
//                  GND
// pressed = HIGH
const bool BUTTON_ACTIVE_HIGH = true;
const bool BUTTON_USE_INTERNAL_PULLUP = false; // keep false for active-HIGH button with external pull-down

// If you rewire later as button to GND with internal pullup:
//   BUTTON_ACTIVE_HIGH = false
//   BUTTON_USE_INTERNAL_PULLUP = true

// ---------------------------
// HOLD/LATCH LOGIC
// ---------------------------
// Your latch logic:
//   HOLD HIGH = power stays ON
//   HOLD LOW  = power turns OFF
const byte HOLD_ON_LEVEL  = HIGH;
const byte HOLD_OFF_LEVEL = LOW;

// IMPORTANT for external D2 pull-up:
// D2 pull-up must go to the Arduino switched +5V, not to an always-on 5V rail.
// If it is connected to always-on 5V, the latch may never turn off.

// ---------------------------
// VOLTAGE CALIBRATION
// ---------------------------
const float dividerRatio = 0.18334;
const float vRef = 5.05;

// ---------------------------
// ACS712 SETTINGS
// ---------------------------
const float acsZeroAdc = 501.8;
const float acsCountsPerAmp = 5.6;
const float currentDeadbandA = 0.05;

// ---------------------------
// PROTECTION THRESHOLDS
// ---------------------------
const float cutOffVoltage    = 16.5;
const float overCurrentLimit = 25.0;

// ---------------------------
// TIMINGS
// ---------------------------
const unsigned long buttonDebounceMs      = 60;
const unsigned long emergencyHoldMs       = 5000;
const unsigned long buttonHoldReportMs    = 500;

const unsigned long normalShutdownGraceMs = 30000;
const unsigned long minShutdownGraceMs    = 5000;
const unsigned long maxShutdownGraceMs    = 180000;

const unsigned long lowVoltageConfirmMs   = 1800;
const unsigned long overCurrentConfirmMs  = 180;
const unsigned long lowVoltageIgnoreMs    = 5000;
const unsigned long overCurrentIgnoreMs   = 3000;

const unsigned long sampleIntervalMs      = 250;
const unsigned long serialIntervalMs      = 1000;
const unsigned long ledBlinkRunMs         = 500;
const unsigned long ledBlinkPendingMs     = 150;
const unsigned long ledBlinkCutMs         = 70;

const byte adcSamplesVoltage = 12;
const byte adcSamplesCurrent = 40;

// ---------------------------
// EEPROM LOG - compact, no char arrays
// ---------------------------
const uint32_t LOG_MAGIC = 0x50575236UL; // PWR6
const byte LOG_VERSION = 6;

struct ShutdownLog {
  uint32_t magic;
  byte version;
  uint32_t sequence;
  uint32_t timestamp_ms;
  byte reason;
  byte pi_ack;
  byte completed;
  uint16_t vbat_mV;
  uint16_t current_cA;
  uint16_t checksum;
};

ShutdownLog lastLog;
uint32_t logSequence = 0;

// ---------------------------
// STATE
// ---------------------------
enum PowerState : byte {
  STATE_RUN = 0,
  STATE_SHUTDOWN_PENDING = 1,
  STATE_POWER_CUT = 2
};

PowerState powerState = STATE_RUN;

// Reason codes
enum ReasonCode : byte {
  REASON_NONE = 0,
  REASON_BUTTON_PRESS = 1,
  REASON_LOW_BATTERY = 2,
  REASON_OVERCURRENT = 3,
  REASON_EMERGENCY_BUTTON_HOLD = 4,
  REASON_EMERGENCY_BUTTON_HOLD_BOOT = 5,
  REASON_SERIAL_CUT_NOW = 6
};

byte pendingReason = REASON_NONE;

bool piReady = false;
bool piShutdownAck = false;
bool shuttingDown = false;
bool ledState = false;

unsigned long startupTime = 0;
unsigned long lastSampleTime = 0;
unsigned long lastSerialTime = 0;
unsigned long lastLedToggleTime = 0;
unsigned long lowVoltageStartTime = 0;
unsigned long overCurrentStartTime = 0;
unsigned long shutdownDeadlineTime = 0;

// Button debounce state
bool buttonRawPressed = false;
bool buttonLastRawPressed = false;
bool buttonStablePressed = false;
bool buttonLastStablePressed = false;
unsigned long buttonRawChangedTime = 0;
unsigned long buttonPressStartTime = 0;
unsigned long lastButtonHoldReportTime = 0;
bool bootButtonGuard = false;
bool emergencyTriggered = false;

// Measurements
float batteryVoltage = 0.0;
float filteredBatteryVoltage = 0.0;
float currentAmps = 0.0;
float filteredCurrentAmps = 0.0;
float lastBatteryPinVoltage = 0.0;
float lastCurrentAdc = 0.0;

// Serial input
char serialBuffer[64];
byte serialLen = 0;
bool lastWasLineEnd = false;

// =====================================================
// PRINT HELPERS - use F() to save SRAM
// =====================================================

void printStateName() {
  if (powerState == STATE_RUN) Serial.print(F("RUN"));
  else if (powerState == STATE_SHUTDOWN_PENDING) Serial.print(F("SHUTDOWN_PENDING"));
  else if (powerState == STATE_POWER_CUT) Serial.print(F("POWER_CUT"));
  else Serial.print(F("UNKNOWN"));
}

void printReason(byte r) {
  switch (r) {
    case REASON_BUTTON_PRESS: Serial.print(F("BUTTON_PRESS")); break;
    case REASON_LOW_BATTERY: Serial.print(F("LOW_BATTERY")); break;
    case REASON_OVERCURRENT: Serial.print(F("OVERCURRENT")); break;
    case REASON_EMERGENCY_BUTTON_HOLD: Serial.print(F("EMERGENCY_BUTTON_HOLD")); break;
    case REASON_EMERGENCY_BUTTON_HOLD_BOOT: Serial.print(F("EMERGENCY_BUTTON_HOLD_BOOT")); break;
    case REASON_SERIAL_CUT_NOW: Serial.print(F("SERIAL_CUT_NOW")); break;
    default: Serial.print(F("NONE")); break;
  }
}

// =====================================================
// BASIC HARDWARE HELPERS
// =====================================================

bool readButtonRawPressed() {
  int v = digitalRead(buttonPin);
  if (BUTTON_ACTIVE_HIGH) return v == HIGH;
  return v == LOW;
}

void setHold(bool on) {
  pinMode(holdPin, OUTPUT);
  digitalWrite(holdPin, on ? HOLD_ON_LEVEL : HOLD_OFF_LEVEL);
}

void forceHoldOff() {
  pinMode(holdPin, OUTPUT);
  digitalWrite(holdPin, HOLD_OFF_LEVEL);
}

unsigned long getButtonHeldMs() {
  if (!buttonStablePressed) return 0;
  if (buttonPressStartTime == 0) return 0;
  return millis() - buttonPressStartTime;
}

// =====================================================
// EEPROM LOG
// =====================================================

uint16_t calcChecksum(const ShutdownLog &log) {
  const byte *p = (const byte*)&log;
  uint16_t sum = 0xA55A;
  for (unsigned int i = 0; i < sizeof(ShutdownLog) - sizeof(uint16_t); i++) {
    sum ^= p[i];
    sum = (sum << 1) | (sum >> 15);
  }
  return sum;
}

bool isValidLog(const ShutdownLog &log) {
  if (log.magic != LOG_MAGIC) return false;
  if (log.version != LOG_VERSION) return false;
  return calcChecksum(log) == log.checksum;
}

void readPreviousLog() {
  EEPROM.get(0, lastLog);
  if (isValidLog(lastLog)) logSequence = lastLog.sequence;
  else logSequence = 0;
}

void writeShutdownLog(byte reason, bool piAck, bool completed) {
  ShutdownLog log;
  memset(&log, 0, sizeof(log));

  log.magic = LOG_MAGIC;
  log.version = LOG_VERSION;
  log.sequence = logSequence + 1;
  log.timestamp_ms = millis();
  log.reason = reason;
  log.pi_ack = piAck ? 1 : 0;
  log.completed = completed ? 1 : 0;

  float vb = filteredBatteryVoltage;
  if (vb < 0.01f) vb = batteryVoltage;
  if (vb < 0.0f) vb = 0.0f;
  if (vb > 65.0f) vb = 65.0f;

  float ca = filteredCurrentAmps;
  if (ca < 0.01f) ca = currentAmps;
  if (ca < 0.0f) ca = 0.0f;
  if (ca > 650.0f) ca = 650.0f;

  log.vbat_mV = (uint16_t)(vb * 1000.0f);
  log.current_cA = (uint16_t)(ca * 100.0f);
  log.checksum = calcChecksum(log);

  EEPROM.put(0, log);
  lastLog = log;
  logSequence = log.sequence;
}

void printPreviousLog() {
  if (!isValidLog(lastLog)) {
    Serial.println(F("PWRPREV valid=0 reason=NO_VALID_LOG"));
    return;
  }

  Serial.print(F("PWRPREV valid=1 version="));
  Serial.print(lastLog.version);
  Serial.print(F(" seq="));
  Serial.print(lastLog.sequence);
  Serial.print(F(" reason="));
  printReason(lastLog.reason);
  Serial.print(F(" pi_ack="));
  Serial.print(lastLog.pi_ack);
  Serial.print(F(" completed="));
  Serial.print(lastLog.completed);
  Serial.print(F(" vbat="));
  Serial.print(lastLog.vbat_mV / 1000.0, 2);
  Serial.print(F(" current="));
  Serial.print(lastLog.current_cA / 100.0, 2);
  Serial.print(F(" t_ms="));
  Serial.println(lastLog.timestamp_ms);
}

// =====================================================
// MEASUREMENTS
// =====================================================

int analogReadStable(byte pin) {
  analogRead(pin);
  delayMicroseconds(180);
  return analogRead(pin);
}

float readAverageAdcStable(byte pin, byte samples, byte delayMs) {
  unsigned long sumADC = 0;
  analogRead(pin);
  delayMicroseconds(250);
  for (byte i = 0; i < samples; i++) {
    sumADC += analogReadStable(pin);
    delay(delayMs);
  }
  return sumADC / (float)samples;
}

float readBatteryVoltageRaw() {
  float avgADC = readAverageAdcStable(batVoltPin, adcSamplesVoltage, 1);
  float pinVoltage = (avgADC * vRef) / 1023.0;
  lastBatteryPinVoltage = pinVoltage;
  return pinVoltage / dividerRatio;
}

float readBatteryVoltageFiltered() {
  float newValue = readBatteryVoltageRaw();
  if (filteredBatteryVoltage < 0.01f) filteredBatteryVoltage = newValue;
  else filteredBatteryVoltage = filteredBatteryVoltage * 0.85f + newValue * 0.15f;
  batteryVoltage = newValue;
  return filteredBatteryVoltage;
}

float readCurrentRaw() {
  float avgADC = readAverageAdcStable(currPin, adcSamplesCurrent, 1);
  lastCurrentAdc = avgADC;

  float deltaCounts = avgADC - acsZeroAdc;
  if (deltaCounts < 0) deltaCounts = -deltaCounts;

  float amps = deltaCounts / acsCountsPerAmp;
  if (amps < currentDeadbandA) amps = 0.0;
  return amps;
}

float readCurrentFiltered() {
  float newValue = readCurrentRaw();
  if (filteredCurrentAmps < 0.01f) filteredCurrentAmps = newValue;
  else filteredCurrentAmps = filteredCurrentAmps * 0.75f + newValue * 0.25f;
  currentAmps = newValue;
  return filteredCurrentAmps;
}

// =====================================================
// STATUS
// =====================================================

void printStatus() {
  unsigned long now = millis();
  long rem = -1;
  if (powerState == STATE_SHUTDOWN_PENDING) {
    rem = (long)(shutdownDeadlineTime - now);
    if (rem < 0) rem = 0;
  }

  Serial.print(F("PWRSTAT state="));
  printStateName();
  Serial.print(F(" vbat_raw="));
  Serial.print(batteryVoltage, 2);
  Serial.print(F(" vbat_filt="));
  Serial.print(filteredBatteryVoltage, 2);
  Serial.print(F(" a0="));
  Serial.print(lastBatteryPinVoltage, 3);
  Serial.print(F(" acs="));
  Serial.print(lastCurrentAdc, 1);
  Serial.print(F(" i_raw="));
  Serial.print(currentAmps, 2);
  Serial.print(F(" i_filt="));
  Serial.print(filteredCurrentAmps, 2);
  Serial.print(F(" btn_raw="));
  Serial.print(buttonRawPressed ? 1 : 0);
  Serial.print(F(" btn="));
  Serial.print(buttonStablePressed ? 1 : 0);
  Serial.print(F(" hold_ms="));
  Serial.print(getButtonHeldMs());
  Serial.print(F(" hold="));
  Serial.print(digitalRead(holdPin) ? 1 : 0);
  Serial.print(F(" boot_guard="));
  Serial.print(bootButtonGuard ? 1 : 0);
  Serial.print(F(" pi_ready="));
  Serial.print(piReady ? 1 : 0);
  Serial.print(F(" pi_ack="));
  Serial.print(piShutdownAck ? 1 : 0);
  Serial.print(F(" reason="));
  printReason(pendingReason);
  Serial.print(F(" rem_ms="));
  Serial.println(rem);
}

// =====================================================
// POWER ACTIONS
// =====================================================

void holdOffForeverBlink() {
  while (true) {
    forceHoldOff();
    digitalWrite(ledPin, HIGH);
    delay(ledBlinkCutMs);
    forceHoldOff();
    digitalWrite(ledPin, LOW);
    delay(ledBlinkCutMs);
  }
}

void cutPowerNow(byte reason, bool piAck, bool completed) {
  if (shuttingDown) return;
  shuttingDown = true;
  powerState = STATE_POWER_CUT;
  pendingReason = reason;

  writeShutdownLog(reason, piAck, completed);

  Serial.print(F("PWREVT type=POWER_CUT reason="));
  printReason(reason);
  Serial.print(F(" pi_ack="));
  Serial.print(piAck ? 1 : 0);
  Serial.print(F(" completed="));
  Serial.println(completed ? 1 : 0);
  Serial.flush();

  // Drive HOLD low before entering the blink loop.
  forceHoldOff();
  delay(20);
  forceHoldOff();

  holdOffForeverBlink();
}

void requestGracefulShutdown(byte reason, unsigned long graceMs) {
  if (shuttingDown) return;
  if (powerState == STATE_SHUTDOWN_PENDING) return;

  if (graceMs < minShutdownGraceMs) graceMs = minShutdownGraceMs;
  if (graceMs > maxShutdownGraceMs) graceMs = maxShutdownGraceMs;

  powerState = STATE_SHUTDOWN_PENDING;
  pendingReason = reason;
  piShutdownAck = false;
  shutdownDeadlineTime = millis() + graceMs;

  writeShutdownLog(reason, false, false);

  Serial.print(F("PWREVT type=SHUTDOWN_REQUEST reason="));
  printReason(reason);
  Serial.print(F(" grace_ms="));
  Serial.println(graceMs);
  Serial.flush();
}

void cancelGracefulShutdown() {
  if (powerState != STATE_SHUTDOWN_PENDING) {
    Serial.print(F("PWRACK cmd=CANCEL_SHUTDOWN ignored=1 state="));
    printStateName();
    Serial.println();
    return;
  }

  powerState = STATE_RUN;
  pendingReason = REASON_NONE;
  piShutdownAck = false;
  shutdownDeadlineTime = 0;
  Serial.println(F("PWRACK cmd=CANCEL_SHUTDOWN"));
}

// =====================================================
// SERIAL PROTOCOL - CR or LF both work
// =====================================================

void handleSerialCommand(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  if (strcmp(line, "PI_READY") == 0) {
    piReady = true;
    Serial.println(F("PWRACK cmd=PI_READY"));
    printPreviousLog();
    printStatus();
    return;
  }

  if (strncmp(line, "PI_SHUTTING_DOWN", 16) == 0) {
    piShutdownAck = true;
    unsigned long requestedGrace = normalShutdownGraceMs;

    char *arg = line + 16;
    while (*arg == ' ' || *arg == '\t') arg++;
    if (*arg >= '0' && *arg <= '9') {
      requestedGrace = strtoul(arg, NULL, 10);
      if (requestedGrace < minShutdownGraceMs) requestedGrace = minShutdownGraceMs;
      if (requestedGrace > maxShutdownGraceMs) requestedGrace = maxShutdownGraceMs;
    }

    if (powerState == STATE_SHUTDOWN_PENDING) {
      shutdownDeadlineTime = millis() + requestedGrace;
      writeShutdownLog(pendingReason, true, false);
    }

    Serial.print(F("PWRACK cmd=PI_SHUTTING_DOWN grace_ms="));
    Serial.print(requestedGrace);
    Serial.print(F(" state="));
    printStateName();
    Serial.println();
    return;
  }

  if (strcmp(line, "CANCEL_SHUTDOWN") == 0) {
    cancelGracefulShutdown();
    return;
  }

  if (strcmp(line, "STATUS?") == 0) {
    printPreviousLog();
    printStatus();
    return;
  }

  if (strcmp(line, "CUT_NOW") == 0) {
    cutPowerNow(REASON_SERIAL_CUT_NOW, piShutdownAck, true);
    return;
  }

  Serial.print(F("PWRERR unknown_cmd="));
  Serial.println(line);
}

void finishSerialLine() {
  if (serialLen == 0) return;
  serialBuffer[serialLen] = '\0';
  handleSerialCommand(serialBuffer);
  serialLen = 0;
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (!lastWasLineEnd) finishSerialLine();
      lastWasLineEnd = true;
      continue;
    }

    lastWasLineEnd = false;

    if (serialLen < sizeof(serialBuffer) - 1) {
      serialBuffer[serialLen++] = c;
    } else {
      serialLen = 0;
      Serial.println(F("PWRERR serial_overflow=1"));
    }
  }
}

// =====================================================
// BUTTON HANDLING
// =====================================================

void updateButtonDebounce() {
  unsigned long now = millis();
  buttonRawPressed = readButtonRawPressed();

  if (buttonRawPressed != buttonLastRawPressed) {
    buttonLastRawPressed = buttonRawPressed;
    buttonRawChangedTime = now;
  }

  if ((now - buttonRawChangedTime) >= buttonDebounceMs) {
    buttonStablePressed = buttonRawPressed;
  }
}

void reportButtonHold(unsigned long heldMs) {
  unsigned long now = millis();
  if (now - lastButtonHoldReportTime < buttonHoldReportMs) return;
  lastButtonHoldReportTime = now;

  long toEmergency = (long)emergencyHoldMs - (long)heldMs;
  if (toEmergency < 0) toEmergency = 0;

  Serial.print(F("PWREVT type=BUTTON_HOLD held_ms="));
  Serial.print(heldMs);
  Serial.print(F(" emergency_in_ms="));
  Serial.println(toEmergency);
}

void handleButton() {
  unsigned long now = millis();

  buttonLastStablePressed = buttonStablePressed;
  updateButtonDebounce();

  // Press edge
  if (buttonStablePressed && !buttonLastStablePressed) {
    buttonPressStartTime = now;
    lastButtonHoldReportTime = 0;
    emergencyTriggered = false;
    Serial.println(F("PWREVT type=BUTTON_DOWN"));
  }

  // Release edge
  if (!buttonStablePressed && buttonLastStablePressed) {
    unsigned long heldMs = 0;
    if (buttonPressStartTime != 0) heldMs = now - buttonPressStartTime;

    Serial.print(F("PWREVT type=BUTTON_UP held_ms="));
    Serial.println(heldMs);

    bool wasBootGuard = bootButtonGuard;
    bootButtonGuard = false;
    buttonPressStartTime = 0;
    lastButtonHoldReportTime = 0;

    // Do not treat the same press used for power-on as a shutdown request.
    if (!wasBootGuard && !emergencyTriggered && heldMs >= buttonDebounceMs && heldMs < emergencyHoldMs) {
      requestGracefulShutdown(REASON_BUTTON_PRESS, normalShutdownGraceMs);
    }
  }

  // Held
  if (buttonStablePressed && buttonPressStartTime != 0) {
    unsigned long heldMs = now - buttonPressStartTime;
    reportButtonHold(heldMs);

    if (!emergencyTriggered && heldMs >= emergencyHoldMs) {
      emergencyTriggered = true;
      if (bootButtonGuard) cutPowerNow(REASON_EMERGENCY_BUTTON_HOLD_BOOT, false, true);
      else cutPowerNow(REASON_EMERGENCY_BUTTON_HOLD, piShutdownAck, true);
    }
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  if (BUTTON_USE_INTERNAL_PULLUP) pinMode(buttonPin, INPUT_PULLUP);
  else pinMode(buttonPin, INPUT);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Immediately take over latch.
  setHold(true);

  startupTime = millis();

  delay(30);
  buttonRawPressed = readButtonRawPressed();
  buttonLastRawPressed = buttonRawPressed;
  buttonStablePressed = buttonRawPressed;
  buttonLastStablePressed = buttonStablePressed;
  buttonRawChangedTime = millis();

  if (buttonStablePressed) {
    bootButtonGuard = true;
    buttonPressStartTime = millis();
  }

  readPreviousLog();

  Serial.println();
  Serial.println(F("PWRBOOT version=6 baud=115200 latch=ON"));
  Serial.print(F("PWRBOOT btn_active_high="));
  Serial.print(BUTTON_ACTIVE_HIGH ? 1 : 0);
  Serial.print(F(" internal_pullup="));
  Serial.print(BUTTON_USE_INTERNAL_PULLUP ? 1 : 0);
  Serial.print(F(" boot_btn="));
  Serial.print(bootButtonGuard ? 1 : 0);
  Serial.print(F(" emergency_ms="));
  Serial.println(emergencyHoldMs);

  readBatteryVoltageFiltered();
  readBatteryVoltageFiltered();
  readCurrentFiltered();
  readCurrentFiltered();

  printPreviousLog();
  printStatus();

  lastSampleTime = millis();
  lastSerialTime = millis();
  lastLedToggleTime = millis();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  if (shuttingDown) holdOffForeverBlink();

  unsigned long now = millis();

  pollSerial();
  handleButton();

  // LED heartbeat
  unsigned long blinkMs = (powerState == STATE_SHUTDOWN_PENDING) ? ledBlinkPendingMs : ledBlinkRunMs;
  if (now - lastLedToggleTime >= blinkMs) {
    lastLedToggleTime = now;
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
  }

  // Measurements/protection
  if (now - lastSampleTime >= sampleIntervalMs) {
    lastSampleTime = now;

    readBatteryVoltageFiltered();
    readCurrentFiltered();

    if (powerState == STATE_RUN && now - startupTime > lowVoltageIgnoreMs) {
      if (filteredBatteryVoltage < cutOffVoltage) {
        if (lowVoltageStartTime == 0) lowVoltageStartTime = now;
        else if (now - lowVoltageStartTime >= lowVoltageConfirmMs) {
          requestGracefulShutdown(REASON_LOW_BATTERY, normalShutdownGraceMs);
        }
      } else {
        lowVoltageStartTime = 0;
      }
    }

    if (powerState == STATE_RUN && now - startupTime > overCurrentIgnoreMs) {
      if (filteredCurrentAmps > overCurrentLimit) {
        if (overCurrentStartTime == 0) overCurrentStartTime = now;
        else if (now - overCurrentStartTime >= overCurrentConfirmMs) {
          cutPowerNow(REASON_OVERCURRENT, false, true);
        }
      } else {
        overCurrentStartTime = 0;
      }
    }
  }

  // Graceful shutdown timeout -> power cut
  if (powerState == STATE_SHUTDOWN_PENDING) {
    if ((long)(now - shutdownDeadlineTime) >= 0) {
      cutPowerNow(pendingReason, piShutdownAck, true);
    }
  }

  // Periodic status
  if (now - lastSerialTime >= serialIntervalMs) {
    lastSerialTime = now;
    printStatus();
  }
}
