// Host-side smoke tests for the core modules (no Arduino, no hardware).
// Build and run:
//   g++ -std=c++11 -I ../shipguard test_core.cpp ../shipguard/crc8.cpp \
//       ../shipguard/frame.cpp ../shipguard/features.cpp ../shipguard/risk.cpp \
//       ../shipguard/fsm.cpp -o test_core && ./test_core
//
// This is intentionally a plain assert() harness rather than a framework:
// the goal is a fast correctness check the student can re-run after any
// change to crc8/frame/features/risk/fsm, not full coverage.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "crc8.h"
#include "frame.h"
#include "features.h"
#include "risk.h"
#include "fsm.h"

static int g_failures = 0;
#define CHECK(cond)                                                         \
  do {                                                                      \
    if (!(cond)) {                                                          \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                \
      g_failures++;                                                         \
    }                                                                       \
  } while (0)

static void test_crc8_roundtrip() {
  uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
  uint8_t crc = crc8_buffer(data, sizeof(data));
  // Same input must always give the same CRC, and flipping one bit must
  // change it (this is not a proof of the polynomial, just a sanity check
  // that the implementation is deterministic and actually looks at the data).
  CHECK(crc == crc8_buffer(data, sizeof(data)));
  data[2] ^= 0x01;
  CHECK(crc != crc8_buffer(data, sizeof(data)));
}

static void test_frame_build_and_parse() {
  TelemetryPayload p;
  memset(&p, 0, sizeof(p));
  p.t_s = 12345;
  p.state = SG_STATE_MONITORING;
  p.level = RISK_LEVEL_L1;
  p.profile = PROFILE_GLASS;
  p.flags = SG_FLAG_PIR;
  p.risk = 55;
  p.peak_mg = 3200;
  p.dv_mms = 900;
  p.vib_rms_mg = 0;
  p.tilt_x10 = 123;
  p.temp_x100 = 2350;
  p.hum_x100 = 5100;
  p.press_x10 = 10136;

  uint8_t payload[SG_PAYLOAD_MAX];
  telemetry_pack(&p, payload);

  uint8_t frame[SG_FRAME_MAX];
  uint8_t n = frame_build(SG_TYPE_TELEMETRY, 7, payload, SG_PAYLOAD_MAX, frame);
  CHECK(n == 5 + SG_PAYLOAD_MAX);
  CHECK(frame[0] == SG_SOF_TO_PC);
  CHECK(frame[1] == SG_PAYLOAD_MAX);
  CHECK(frame[2] == SG_TYPE_TELEMETRY);
  CHECK(frame[3] == 7);

  // Feed the same bytes back through the parser as if they arrived on the
  // *node's* input, just with the other SOF byte, to check the decode path.
  frame[0] = SG_SOF_TO_NODE;
  uint8_t recomputed_crc = crc8_buffer(&frame[1], 3 + SG_PAYLOAD_MAX);
  frame[4 + SG_PAYLOAD_MAX] = recomputed_crc;

  FrameParser fp;
  frame_parser_init(&fp);
  ParsedFrame out;
  uint8_t got = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (frame_parser_feed(&fp, frame[i], &out)) got = 1;
  }
  CHECK(got == 1);
  CHECK(out.type == SG_TYPE_TELEMETRY);
  CHECK(out.payload_len == SG_PAYLOAD_MAX);
  CHECK(memcmp(out.payload, payload, SG_PAYLOAD_MAX) == 0);
}

static void test_frame_parser_rejects_bad_crc() {
  uint8_t payload[1] = {0xAB};
  uint8_t frame[SG_FRAME_MAX];
  uint8_t n = frame_build(SG_TYPE_EVENT, 3, payload, 1, frame);
  frame[0] = SG_SOF_TO_NODE;  // pretend this arrived at the node
  frame[4] ^= 0xFF;           // corrupt the payload byte; CRC byte is now stale

  FrameParser fp;
  frame_parser_init(&fp);
  ParsedFrame out;
  uint8_t got = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (frame_parser_feed(&fp, frame[i], &out)) got = 1;
  }
  CHECK(got == 0);  // corrupted frame must not validate

  // Parser must have resynced and accept a clean frame right after.
  uint8_t good[SG_FRAME_MAX];
  uint8_t gn = frame_build(SG_TYPE_EVENT, 4, payload, 1, good);
  good[0] = SG_SOF_TO_NODE;
  uint8_t good_crc = crc8_buffer(&good[1], 3 + 1);
  good[4 + 1] = good_crc;
  got = 0;
  for (uint8_t i = 0; i < gn; i++) {
    if (frame_parser_feed(&fp, good[i], &out)) got = 1;
  }
  CHECK(got == 1);
  CHECK(out.type == SG_TYPE_EVENT);
}

static void test_shock_detector_basic() {
  ShockDetector sd;
  shock_init(&sd);
  AccelMg rest = {0, 0, 1000};  // resting flat, ~1 g on Z
  ShockEvent ev;
  for (int i = 0; i < 5; i++) {
    ev = shock_update(&sd, &rest);
    CHECK(ev.event_ended == 0);
  }
  // A hard tap: well above SHOCK_START_MG on one axis, saturating briefly.
  AccelMg tap = {0, 0, 1000 + 5000};
  ev = shock_update(&sd, &tap);
  CHECK(ev.event_ended == 0);
  CHECK(sd.active == 1);
  // Quiet samples close the event after SHOCK_END_QUIET_SAMPLES.
  for (int i = 0; i < (int)SHOCK_END_QUIET_SAMPLES; i++) {
    ev = shock_update(&sd, &rest);
  }
  CHECK(ev.event_ended == 1);
  CHECK(ev.peak_mg >= 4900 && ev.peak_mg <= 5100);
  CHECK(ev.saturated == 0);
  CHECK(sd.active == 0);  // detector reset itself
}

static void test_shock_detector_saturation() {
  ShockDetector sd;
  shock_init(&sd);
  AccelMg sat = {17000, 0, 0};  // beyond +-16 g full scale on X
  ShockEvent ev = shock_update(&sd, &sat);
  (void)ev;
  AccelMg rest = {0, 0, 1000};
  for (int i = 0; i < (int)SHOCK_END_QUIET_SAMPLES; i++) ev = shock_update(&sd, &rest);
  CHECK(ev.event_ended == 1);
  CHECK(ev.saturated == 1);
}

static void test_tilt_static_gate_and_dwell() {
  TiltTracker tt;
  tilt_init(&tt);
  AccelMg flat = {0, 0, 1000};
  tilt_calibrate(&tt, &flat);

  int16_t angle = tilt_update(&tt, &flat, 1.0f, 300 /* 30.0 deg limit */);
  CHECK(angle == 0);  // dead flat should read (near) zero, well under the deadband
  CHECK(tt.dwell_s == 0);

  // Tilt hard past the limit while stationary (small |a| so it clears the
  // static gate): roughly 45 degrees from vertical.
  AccelMg tilted = {707, 0, 707};
  for (int i = 0; i < 3; i++) angle = tilt_update(&tt, &tilted, 1.0f, 300);
  CHECK(angle > 300);          // past the 30 deg limit
  CHECK(tt.dwell_s > 0.0f);    // dwell accumulating

  // A violent sample (big |a|, fails the static gate) must freeze, not reset.
  AccelMg moving = {0, 0, 6000};
  float dwell_before = tt.dwell_s;
  int16_t angle_before = angle;
  angle = tilt_update(&tt, &moving, 1.0f, 300);
  CHECK(angle == angle_before);
  CHECK(tt.dwell_s == dwell_before);
}

static void test_risk_score_and_hysteresis() {
  RiskState rs;
  risk_init(&rs);
  const Profile *glass = &PROFILES[PROFILE_GLASS];

  // A shock at exactly g_crit should score ~100 (raw, before display clamp).
  risk_on_shock_event(&rs, glass, glass->g_crit_mg, 0, 0);
  uint16_t s = risk_tick_1s(&rs, glass, 0, 0.0f);
  CHECK(s >= 99 && s <= 101);

  LevelTracker lt;
  level_tracker_init(&lt);
  uint8_t level = level_tracker_update(&lt, s);
  CHECK(level == RISK_LEVEL_L3);

  // Score drops immediately after decay starts (after the 30 s hold), but
  // the level must not drop until 5 consecutive seconds under (100-10)=90.
  rs.shock_hold_s = 0;
  rs.shock_score = 50;  // simulate having already decayed
  uint16_t low_score = risk_tick_1s(&rs, glass, 0, 0.0f);
  CHECK(low_score < 90);
  for (int i = 0; i < 4; i++) {
    level = level_tracker_update(&lt, low_score);
    CHECK(level == RISK_LEVEL_L3);  // still latched, < 5 s so far
  }
  level = level_tracker_update(&lt, low_score);
  CHECK(level == RISK_LEVEL_L1);  // 5th consecutive second: score 50 -> L1

  // Saturation forces the score high enough for L3 regardless of profile.
  RiskState rs2;
  risk_init(&rs2);
  risk_on_shock_event(&rs2, glass, 100 /* tiny peak_mg */, 10, /*saturated=*/1);
  uint16_t s2 = risk_tick_1s(&rs2, glass, 0, 0.0f);
  CHECK(s2 >= 100);
}

static void test_fsm_latches_and_requires_ack() {
  FsmState fsm;
  fsm_init(&fsm);
  CHECK(fsm.state == SG_STATE_MONITORING);

  fsm_tick(&fsm, RISK_LEVEL_L2, /*shock_driven=*/1);
  CHECK(fsm.state == SG_STATE_ALERT);

  // Shock alert: level drops back to L0 but no ACK yet, must stay in ALERT.
  fsm_tick(&fsm, RISK_LEVEL_L0, 0);
  CHECK(fsm.state == SG_STATE_ALERT);

  fsm_ack(&fsm);
  // acked=1 recorded, but the transition only happens on the next tick.
  fsm_tick(&fsm, RISK_LEVEL_L0, 0);
  CHECK(fsm.state == SG_STATE_MONITORING);
  CHECK(fsm.acked == 0);  // consumed
}

static void test_fsm_ack_rules() {
  FsmState fsm;
  fsm_init(&fsm);

  // A stray press in MONITORING is ignored, so it cannot pre-silence the
  // next alert.
  CHECK(fsm_ack(&fsm) == 0);
  CHECK(fsm.acked == 0);
  fsm_tick(&fsm, RISK_LEVEL_L3, 1);
  CHECK(fsm.state == SG_STATE_ALERT);
  CHECK(fsm.acked == 0);

  // First ACK is accepted, a second one is a no-op (one ACK_LOG per alert).
  CHECK(fsm_ack(&fsm) == 1);
  CHECK(fsm_ack(&fsm) == 0);
  CHECK(fsm.acked == 1);

  // Still at L3 after the ACK: stays in ALERT, silenced.
  fsm_tick(&fsm, RISK_LEVEL_L3, 1);
  CHECK(fsm.state == SG_STATE_ALERT);
  CHECK(fsm.acked == 1);

  // A new hazard re-arms the alarm and needs a fresh ACK.
  fsm_rearm(&fsm);
  CHECK(fsm.acked == 0);
  CHECK(fsm_ack(&fsm) == 1);

  // rearm outside ALERT does nothing.
  fsm_tick(&fsm, RISK_LEVEL_L0, 0);
  CHECK(fsm.state == SG_STATE_MONITORING);
  fsm_rearm(&fsm);
  CHECK(fsm.acked == 0);
}

static void test_fsm_tilt_alert_self_clears() {
  FsmState fsm;
  fsm_init(&fsm);
  // Tilt-only alert (shock part below L2).
  fsm_tick(&fsm, RISK_LEVEL_L2, 0);
  CHECK(fsm.state == SG_STATE_ALERT);
  CHECK(fsm.latched == 0);
  // L1 is not enough to clear an unacknowledged alert.
  fsm_tick(&fsm, RISK_LEVEL_L1, 0);
  CHECK(fsm.state == SG_STATE_ALERT);
  // Back at L0 without any ACK: the condition is gone, the alert clears.
  fsm_tick(&fsm, RISK_LEVEL_L0, 0);
  CHECK(fsm.state == SG_STATE_MONITORING);

  // A tilt alert that is joined by a shock becomes latched.
  fsm_tick(&fsm, RISK_LEVEL_L2, 0);
  fsm_tick(&fsm, RISK_LEVEL_L3, 1);
  CHECK(fsm.latched == 1);
  fsm_tick(&fsm, RISK_LEVEL_L0, 0);
  CHECK(fsm.state == SG_STATE_ALERT);
}

static void test_risk_event_returns_own_score() {
  const Profile *glass = &PROFILES[PROFILE_GLASS];
  RiskState rs;
  risk_init(&rs);
  CHECK(risk_on_shock_event(&rs, glass, glass->g_crit_mg, 0, 0) == 100);
  // A smaller bump returns its own (small) score even though the latched
  // shock score stays at 100.
  CHECK(risk_on_shock_event(&rs, glass, glass->g_crit_mg / 10, 0, 0) == 10);
  CHECK(rs.shock_score == 100);
  CHECK(risk_on_shock_event(&rs, glass, 100, 10, /*saturated=*/1) == 999);
}

int main() {
  test_crc8_roundtrip();
  test_frame_build_and_parse();
  test_frame_parser_rejects_bad_crc();
  test_shock_detector_basic();
  test_shock_detector_saturation();
  test_tilt_static_gate_and_dwell();
  test_risk_score_and_hysteresis();
  test_fsm_latches_and_requires_ack();
  test_fsm_ack_rules();
  test_fsm_tilt_alert_self_clears();
  test_risk_event_returns_own_score();

  if (g_failures == 0) {
    printf("All tests passed.\n");
    return 0;
  }
  printf("%d check(s) failed.\n", g_failures);
  return 1;
}
