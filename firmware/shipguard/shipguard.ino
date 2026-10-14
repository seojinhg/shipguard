// ShipGuard MVP firmware (Arduino UNO R3 + Arduino Sensor Kit Base + PIR).
// Scope: docs/design.md section 0 (MVP). 2-state FSM (MONITORING/ALERT),
// shock+tilt risk score only (no vibration RMS, no cumulative damage, no
// auto-escalation, no phone push). See docs/design.md for the full design,
// including what Extension 1/2 add on top of this file.
//
// This file is the HAL + scheduler. The testable logic lives in the plain
// C files next to this sketch (crc8, frame, features, risk, fsm) and is
// covered by firmware/tests/test_core.cpp on the host.
//
// Serial: 115200 8N1. Frame format: docs/design.md section 8.
//   h  -> print a one-line status/help line
// Commands accepted from the PC (see frame.h): ACK (0x81), SET_PROFILE
// (0x82, 1-byte profile index 0-2), PING (0x84, replies with a STATUS frame),
// SET_INTERVAL (0x85, 1-byte report period in seconds, 1-60; demo mode uses 1),
// ARM (0x87, new shipment: recalibrate tilt and reset risk/level/FSM).

#include "Arduino_SensorKit.h"
#include "crc8.h"
#include "frame.h"
#include "features.h"
#include "risk.h"
#include "fsm.h"

// ---------------- Pins (confirmed on hardware, see docs/design.md section 1) ----------------
const uint8_t PIN_BUTTON = 4;
const uint8_t PIN_BUZZER = 5;
const uint8_t PIN_LED = 6;
const uint8_t PIN_POT = A0;
const uint8_t PIN_LIGHT = A3;
const uint8_t PIN_PIR = 2;  // INT0, Grove PIR module
const uint8_t ACCEL_ADDR = 0x19;

// The kit's Accelerometer wrapper cannot change the measurement range, so we
// drive the Grove LIS3DHTR driver directly (see docs/design.md section 1).
LIS3DHTR<TwoWire> lis;

// ---------------- Core state (plain C structs, see the matching .h files) ----------------
static ShockDetector shockDet;
static TiltTracker tiltTrk;
static RiskState riskSt;
static LevelTracker levelTrk;
static FsmState fsm;
static uint8_t currentProfileIdx = PROFILE_ELECTRONICS;
// Highest level reached in the current ALERT episode. The buzzer follows this,
// not the live level, so an unacknowledged alarm keeps sounding until the
// driver ACKs even if the score dips back under the L2 threshold.
static uint8_t episodeMaxLevel = 0;

static AccelMg lastAccelMg = {0, 0, 1000};
static int16_t lastTiltX10 = 0;
static uint16_t lastEventPeakMg = 0;
static uint16_t lastEventDvMms = 0;
static uint8_t lastEventSaturated = 0;

// ---------------- Comm ----------------
static FrameParser rxParser;
static uint8_t txSeq = 0;

// ---------------- Environment (DHT20 + BMP280), read only in MONITORING ----------------
static bool envOk = false, bmpOk = false;
static float lastTempC = 0, lastHumPct = 0, lastPressHpa = 0;

// ---------------- Box-open detection (display-only in MVP; see features.h note) --------
static float lightBaseline = 0;
static bool lightBaselineInit = false;
static bool boxOpenFlag = false;

// ---------------- PIR ----------------
// The PIR stands in for "a person (the driver) is at the package". In the
// deployed product the same signal comes from the driver's phone BLE RSSI.
// It never changes the risk score: it is logged (PRESENCE frame, PIR flag on
// every frame) so the dashboard can measure alert -> driver arrival -> ACK
// time and tag shocks as "during handling" vs "unattended (vehicle)".
static volatile bool pirEdge = false;
static unsigned long pirActiveUntilMs = 0;
static bool presenceOn = false;
static void pirIsr() { pirEdge = true; }
// Presence holds for PIR_HOLD_MS after the PIR output last read HIGH. The PIR
// re-triggers every few seconds while someone is around (measured: ~2 s HIGH,
// ~1-2 s LOW), so a hold shorter than that gap makes presence flicker and
// every flicker would look like a new arrival.
static const unsigned long PIR_HOLD_MS = 5000;
static bool pirActive() {
  if (digitalRead(PIN_PIR) == HIGH) pirActiveUntilMs = millis() + PIR_HOLD_MS;
  return (long)(pirActiveUntilMs - millis()) > 0;
}

// ---------------- Scheduler bookkeeping ----------------
static unsigned long lastSampleUs = 0;
static const unsigned long SAMPLE_PERIOD_US = 2500;  // 400 Hz
static unsigned long lastSecondMs = 0;
static unsigned long lastUiMs = 0;
static unsigned long lastEnvMs = 0;
static unsigned long lastReportMs = 0;
static uint16_t deadlineMissCount = 0;
// MONITORING report period. 60 s is the product value (docs/design.md 2.2);
// the PC receiver drops it to 1 s with SET_INTERVAL so the live dashboard moves.
static uint8_t reportPeriodS = 60;

extern int __heap_start, *__brkval;
static int freeRam() {
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// ---------------- I2C bring-up helpers (copied from firmware/smoke_test, validated 2026-09-21) --
static bool i2cBusRecover() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);
  digitalWrite(SCL, HIGH);
  bool wasStuck = (digitalRead(SDA) == LOW);
  for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
  }
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  return wasStuck;
}

static bool i2cAcks(uint8_t addr, uint8_t tries = 5) {
  for (uint8_t i = 0; i < tries; i++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) return true;
    delay(50);
  }
  return false;
}

static AccelMg readAccelMg() {
  float fx, fy, fz;
  lis.getAcceleration(&fx, &fy, &fz);
  AccelMg a;
  a.x = (int16_t)(fx * 1000.0f);
  a.y = (int16_t)(fy * 1000.0f);
  a.z = (int16_t)(fz * 1000.0f);
  return a;
}

static void sendFrame(uint8_t type) {
  TelemetryPayload p;
  p.t_s = millis() / 1000;
  p.state = fsm.state;
  p.level = levelTrk.level;
  p.profile = currentProfileIdx;
  p.flags = 0;
  if (lastEventSaturated) p.flags |= SG_FLAG_SAT;
  if (boxOpenFlag) p.flags |= SG_FLAG_OPEN;
  if (presenceOn) p.flags |= SG_FLAG_PIR;
  if (fsm.state == SG_STATE_ALERT && !fsm.acked) p.flags |= SG_FLAG_ACK_PENDING;
  if (!envOk || !bmpOk) p.flags |= SG_FLAG_SENSOR_FAULT;
  p.risk = riskSt.score;
  p.peak_mg = lastEventPeakMg;
  p.dv_mms = lastEventDvMms;
  p.vib_rms_mg = 0;  // Extension 1
  p.tilt_x10 = lastTiltX10;
  p.temp_x100 = (int16_t)(lastTempC * 100.0f);
  p.hum_x100 = (uint16_t)(lastHumPct * 100.0f);
  p.press_x10 = (uint16_t)(lastPressHpa * 10.0f);

  uint8_t payload[SG_PAYLOAD_MAX];
  telemetry_pack(&p, payload);
  uint8_t frame[SG_FRAME_MAX];
  uint8_t n = frame_build(type, txSeq++, payload, SG_PAYLOAD_MAX, frame);
  Serial.write(frame, n);
}

static void sendStatusFrame() {
  // Small ad hoc payload (not the 24-byte telemetry layout): firmware tag,
  // uptime, free SRAM, deadline-miss count, sensor-fault flag.
  uint8_t payload[8];
  uint32_t uptime = millis() / 1000;
  payload[0] = (uint8_t)(uptime & 0xFF);
  payload[1] = (uint8_t)((uptime >> 8) & 0xFF);
  payload[2] = (uint8_t)((uptime >> 16) & 0xFF);
  payload[3] = (uint8_t)((uptime >> 24) & 0xFF);
  int fr = freeRam();
  payload[4] = (uint8_t)(fr & 0xFF);
  payload[5] = (uint8_t)((fr >> 8) & 0xFF);
  payload[6] = (uint8_t)(deadlineMissCount & 0xFF);
  payload[7] = (uint8_t)((!envOk || !bmpOk) ? 1 : 0);

  uint8_t frame[SG_FRAME_MAX];
  uint8_t n = frame_build(SG_TYPE_STATUS, txSeq++, payload, sizeof(payload), frame);
  Serial.write(frame, n);
}

// ACK_LOG payload (8 B, little-endian): t_s u32, source u8 (0 = button,
// 1 = PC), level u8 at ACK time, risk u16 at ACK time.
static void sendAckLog(uint8_t source) {
  uint8_t payload[8];
  uint32_t t = millis() / 1000;
  payload[0] = (uint8_t)(t & 0xFF);
  payload[1] = (uint8_t)((t >> 8) & 0xFF);
  payload[2] = (uint8_t)((t >> 16) & 0xFF);
  payload[3] = (uint8_t)((t >> 24) & 0xFF);
  payload[4] = source;
  payload[5] = levelTrk.level;
  payload[6] = (uint8_t)(riskSt.score & 0xFF);
  payload[7] = (uint8_t)(riskSt.score >> 8);
  uint8_t frame[SG_FRAME_MAX];
  uint8_t n = frame_build(SG_TYPE_ACK_LOG, txSeq++, payload, sizeof(payload), frame);
  Serial.write(frame, n);
}

static void doAck(uint8_t source) {
  if (fsm_ack(&fsm)) sendAckLog(source);
}

// New shipment (emulates the dispatch QR-scan registration): the box is
// assumed to be resting still, so re-take the gravity reference and clear
// everything that belongs to the previous shipment.
static void armNewShipment() {
  AccelMg cal = readAccelMg();
  tilt_init(&tiltTrk);
  tilt_calibrate(&tiltTrk, &cal);
  lastAccelMg = cal;
  shock_init(&shockDet);
  risk_init(&riskSt);
  level_tracker_init(&levelTrk);
  fsm_init(&fsm);
  episodeMaxLevel = 0;
  lastTiltX10 = 0;
  lastEventPeakMg = 0;
  lastEventDvMms = 0;
  lastEventSaturated = 0;
  lightBaseline = analogRead(PIN_LIGHT);
  boxOpenFlag = false;
  sendFrame(SG_TYPE_TELEMETRY);
}

static void handleCommand(const ParsedFrame &f) {
  switch (f.type) {
    case SG_CMD_ACK:
      doAck(1);
      break;
    case SG_CMD_SET_PROFILE:
      if (f.payload_len >= 1 && f.payload[0] < PROFILE_COUNT) {
        currentProfileIdx = f.payload[0];
      }
      break;
    case SG_CMD_PING:
      sendStatusFrame();
      break;
    case SG_CMD_SET_INTERVAL:
      if (f.payload_len >= 1 && f.payload[0] >= 1 && f.payload[0] <= 60) {
        reportPeriodS = f.payload[0];
      }
      break;
    case SG_CMD_ARM:
      armNewShipment();
      break;
    default:
      break;  // unknown command: ignore rather than guess
  }
}

// ---------------- UI (LED / buzzer / OLED) ----------------
static void updateUi() {
  uint8_t level = levelTrk.level;
  unsigned long now = millis();

  bool alarmActive = (fsm.state == SG_STATE_ALERT && !fsm.acked);

  // LED: steady on while an alarm waits for ACK or the level is L2+, slow
  // blink at L1, off at L0.
  if (alarmActive || level >= RISK_LEVEL_L2) {
    digitalWrite(PIN_LED, HIGH);
  } else if (level == RISK_LEVEL_L1) {
    digitalWrite(PIN_LED, ((now / 600) % 2) == 0 ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED, LOW);
  }

  // Buzzer: latched until ACK. Pattern follows the highest level of this
  // alert episode: continuous for L3, intermittent for L2. An acknowledged
  // alert is silent (the level stays visible on the OLED and dashboard) until
  // a new hazard re-arms it.
  if (!alarmActive) {
    noTone(PIN_BUZZER);
  } else if (episodeMaxLevel >= RISK_LEVEL_L3) {
    tone(PIN_BUZZER, 2000);
  } else if ((now / 500) % 2 == 0) {
    tone(PIN_BUZZER, 2000);
  } else {
    noTone(PIN_BUZZER);
  }

  // OLED: rewrite only the fields that change, no full-screen clear, to keep
  // this task cheap enough not to threaten the 400 Hz sample budget.
  static uint8_t lastShownLevel = 0xFF;
  static uint16_t lastShownScore = 0xFFFF;
  static uint8_t lastShownProfile = 0xFF;
  static uint8_t lastShownState = 0xFF;
  static uint8_t lastShownAcked = 0xFF;
  static int16_t lastShownTilt = 0x7FFF;
  static uint8_t lastShownFlags = 0xFF;

  if (currentProfileIdx != lastShownProfile) {
    // Pad to exactly 16 columns: writing past column 15 is not clipped by U8x8.
    Oled.setCursor(0, 0);
    const char *name = PROFILES[currentProfileIdx].name;
    Oled.print(name);
    for (uint8_t i = (uint8_t)strlen(name); i < 16; i++) Oled.print(' ');
    lastShownProfile = currentProfileIdx;
  }
  if (riskSt.score != lastShownScore) {
    Oled.setCursor(0, 2);
    Oled.print("Risk:");
    Oled.print(riskSt.score);
    Oled.print("    ");
    lastShownScore = riskSt.score;
  }
  int16_t tiltDeg = lastTiltX10 / 10;
  if (tiltDeg != lastShownTilt) {
    Oled.setCursor(0, 3);
    Oled.print("Tilt:");
    Oled.print(tiltDeg);
    Oled.print(" deg   ");
    lastShownTilt = tiltDeg;
  }
  if (level != lastShownLevel) {
    Oled.setCursor(0, 4);
    static const char *LEVEL_TEXT[4] = {"L0 OK", "L1 WATCH", "L2 CHECK", "L3 INSPECT"};
    Oled.print(LEVEL_TEXT[level]);
    Oled.print("      ");
    lastShownLevel = level;
  }
  if (fsm.state != lastShownState || fsm.acked != lastShownAcked) {
    Oled.setCursor(0, 6);
    if (fsm.state == SG_STATE_ALERT) {
      Oled.print(fsm.acked ? "ALERT ACKED " : "ALERT PRESS ");
    } else {
      Oled.print("MONITOR     ");
    }
    lastShownState = fsm.state;
    lastShownAcked = fsm.acked;
  }
  // Bottom line: when someone arrives at an unacknowledged alert, prompt them
  // to ACK; otherwise show the display-only PIR / box-open flags.
  bool promptAck = presenceOn && fsm.state == SG_STATE_ALERT && !fsm.acked;
  uint8_t flags = promptAck ? 0x80 : ((presenceOn ? 1 : 0) | (boxOpenFlag ? 2 : 0));
  if (flags != lastShownFlags) {
    Oled.setCursor(0, 7);
    if (promptAck) {
      Oled.print("PRESS BTN TO ACK");  // exactly 16 columns
    } else {
      Oled.print((flags & 1) ? "PIR " : "    ");
      Oled.print((flags & 2) ? "OPEN" : "    ");
      Oled.print("        ");
    }
    lastShownFlags = flags;
  }
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  bool wasStuck = i2cBusRecover();
  (void)wasStuck;
  Wire.begin();
  delay(150);  // DHT20 needs ~100 ms after power-up

  envOk = i2cAcks(0x38) && Environment_I2C.begin();
  if (i2cAcks(0x77) || i2cAcks(0x76)) {
    bmpOk = Pressure.begin();
  }

  // Pressure is read once at boot so the first frames carry a real value. The
  // DHT20 driver refuses reads less than ~1 s apart, so its first reading is
  // pulled forward by the env task instead (see lastEnvMs below).
  if (bmpOk) lastPressHpa = Pressure.readPressure() / 100.0f;

  Oled.begin();
  Oled.setFlipMode(true);
  Oled.setFont(u8x8_font_amstrad_cpc_extended_r);
  Oled.clear();

  lis.begin(Wire, ACCEL_ADDR);
  delay(100);
  lis.setOutputDataRate(LIS3DHTR_DATARATE_400HZ);
  lis.setFullScaleRange(LIS3DHTR_RANGE_16G);
  lis.setHighSolution(true);

  pinMode(PIN_PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PIR), pirIsr, RISING);

  shock_init(&shockDet);
  tilt_init(&tiltTrk);
  risk_init(&riskSt);
  level_tracker_init(&levelTrk);
  fsm_init(&fsm);
  frame_parser_init(&rxParser);

  // Auto-calibrate the gravity reference: the box is assumed to be resting
  // still right after power-up. There is no ARM command in the MVP; a
  // manual re-calibration path is Extension 1.
  delay(200);
  AccelMg cal = readAccelMg();
  tilt_calibrate(&tiltTrk, &cal);
  lastAccelMg = cal;

  int raw = analogRead(PIN_LIGHT);
  lightBaseline = raw;
  lightBaselineInit = true;

  unsigned long now = millis();
  unsigned long nowUs = micros();
  lastSampleUs = nowUs;
  lastSecondMs = now;
  lastUiMs = now;
  lastEnvMs = now - 8500UL;  // first env reading ~1.5 s after boot, then every 10 s
  lastReportMs = now;

  Serial.println(F("ShipGuard MVP boot ok. Send 'h' for a status line."));
}

// ---------------- loop ----------------
void loop() {
  // ---- comm: never blocks, drains whatever is waiting ----
  while (Serial.available() > 0) {
    int b = Serial.read();
    if (b == 'h' && frame_parser_idle(&rxParser)) {
      Serial.print(F("state="));
      Serial.print(fsm.state);
      Serial.print(F(" level="));
      Serial.print(levelTrk.level);
      Serial.print(F(" score="));
      Serial.print(riskSt.score);
      Serial.print(F(" profile="));
      Serial.println(PROFILES[currentProfileIdx].name);
      continue;
    }
    ParsedFrame pf;
    if (frame_parser_feed(&rxParser, (uint8_t)b, &pf)) {
      handleCommand(pf);
    }
  }

  // ---- sample task: 400 Hz, highest priority ----
  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs >= SAMPLE_PERIOD_US) {
    unsigned long gap = nowUs - lastSampleUs;
    if (gap > SAMPLE_PERIOD_US + 500) deadlineMissCount++;
    lastSampleUs = nowUs;

    AccelMg a = readAccelMg();
    lastAccelMg = a;
    ShockEvent ev = shock_update(&shockDet, &a);
    if (ev.event_ended) {
      lastEventPeakMg = ev.peak_mg;
      lastEventDvMms = ev.dv_mms;
      lastEventSaturated = ev.saturated;
      uint16_t evScore = risk_on_shock_event(&riskSt, &PROFILES[currentProfileIdx],
                                             ev.peak_mg, ev.dv_mms, ev.saturated);
      if (evScore >= 70) fsm_rearm(&fsm);  // new L2+ hazard sounds the alarm again
      sendFrame(SG_TYPE_EVENT);
    }
  }

  // ---- button: edge-triggered ACK with a 50 ms lockout against bounce ----
  static bool lastButton = false;
  static unsigned long lastButtonEdgeMs = 0;
  bool button = digitalRead(PIN_BUTTON) == HIGH;
  if (button && !lastButton && millis() - lastButtonEdgeMs > 50) {
    lastButtonEdgeMs = millis();
    doAck(0);
  }
  lastButton = button;

  // ---- PIR: presence = output HIGH now or within PIR_HOLD_MS of it (the
  // interrupt catches pulses shorter than one loop pass). One PRESENCE frame
  // per visit (when presence starts after >= PIR_HOLD_MS of quiet), not per edge.
  if (pirEdge) {
    pirEdge = false;
    pirActiveUntilMs = millis() + PIR_HOLD_MS;
  }
  bool presenceNow = pirActive();
  if (presenceNow && !presenceOn) {
    presenceOn = true;
    sendFrame(SG_TYPE_PRESENCE);
  } else if (!presenceNow) {
    presenceOn = false;
  }

  // ---- 1 Hz task: tilt, risk score, level, state transitions, profile/light ----
  unsigned long nowMs = millis();
  if (nowMs - lastSecondMs >= 1000) {
    lastSecondMs = nowMs;

    lastTiltX10 = tilt_update(&tiltTrk, &lastAccelMg, 1.0f,
                               PROFILES[currentProfileIdx].tilt_limit_x10);
    uint16_t score = risk_tick_1s(&riskSt, &PROFILES[currentProfileIdx],
                                   lastTiltX10, tiltTrk.dwell_s);
    uint8_t prevLevel = levelTrk.level;
    uint8_t level = level_tracker_update(&levelTrk, score);
    if (level > prevLevel) fsm_rearm(&fsm);
    uint8_t prevState = fsm.state;
    fsm_tick(&fsm, level, riskSt.shock_score >= 70 ? 1 : 0);
    if (fsm.state == SG_STATE_ALERT) {
      if (prevState != SG_STATE_ALERT) episodeMaxLevel = level;
      else if (level > episodeMaxLevel) episodeMaxLevel = level;
    } else {
      episodeMaxLevel = 0;
    }
    if (fsm.state != prevState || level != prevLevel) sendFrame(SG_TYPE_ALERT);

    // The potentiometer only takes effect when it moves into a different zone,
    // so a SET_PROFILE from the PC is not overwritten a second later.
    static uint8_t lastPotZone = 0xFF;
    int potRaw = analogRead(PIN_POT);
    uint8_t potZone = (potRaw < 341) ? PROFILE_GLASS
                      : (potRaw < 682) ? PROFILE_ELECTRONICS
                                       : PROFILE_ROBUST;
    if (potZone != lastPotZone) {
      currentProfileIdx = potZone;
      lastPotZone = potZone;
    }

    if (lightBaselineInit) {
      int lightRaw = analogRead(PIN_LIGHT);
      boxOpenFlag = fabs((float)lightRaw - lightBaseline) > 200.0f;
      if (!boxOpenFlag) lightBaseline += ((float)lightRaw - lightBaseline) * 0.1f;
    }
  }

  // ---- env task: only in MONITORING, every 10 s ----
  if (fsm.state == SG_STATE_MONITORING && nowMs - lastEnvMs >= 10000) {
    lastEnvMs = nowMs;
    if (envOk) {
      lastTempC = Environment_I2C.readTemperature();
      lastHumPct = Environment_I2C.getHumidity();
    }
    if (bmpOk) {
      lastPressHpa = Pressure.readPressure() / 100.0f;
    }
  }

  // ---- ui task: ~5 Hz ----
  if (nowMs - lastUiMs >= 200) {
    lastUiMs = nowMs;
    updateUi();
  }

  // ---- report task: reportPeriodS in MONITORING (60 s by default), at most 5 s in ALERT ----
  unsigned long reportPeriodMs = (unsigned long)reportPeriodS * 1000UL;
  if (fsm.state == SG_STATE_ALERT && reportPeriodMs > 5000UL) reportPeriodMs = 5000UL;
  if (nowMs - lastReportMs >= reportPeriodMs) {
    lastReportMs = nowMs;
    sendFrame(SG_TYPE_TELEMETRY);
  }
}
