// Pure C, no Arduino.h. Fragility profiles and the MVP risk score
// (docs/design.md sections 5.1-5.3). MVP score = max(shock, tilt); vibration
// RMS and cumulative damage are Extension 1.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  const char *name;
  uint16_t g_crit_mg;      // shock peak that maps to a raw score of 100
  uint16_t dv_crit_mms;    // delta-V that maps to a raw score of 100
  int16_t tilt_limit_x10;  // degrees * 10
  uint16_t tilt_dwell_s;   // seconds sustained past tilt_limit for score 100
} Profile;

#define PROFILE_GLASS 0
#define PROFILE_ELECTRONICS 1
#define PROFILE_ROBUST 2
#define PROFILE_COUNT 3

extern const Profile PROFILES[PROFILE_COUNT];

// Alert levels, thresholds from docs/design.md section 5.3.
#define RISK_LEVEL_L0 0
#define RISK_LEVEL_L1 1
#define RISK_LEVEL_L2 2
#define RISK_LEVEL_L3 3

typedef struct {
  uint16_t shock_score;    // decays over time; see risk_tick_1s
  uint16_t shock_hold_s;   // seconds left before decay starts
  uint8_t shock_saturated; // sticky until the next event overwrites it
  uint16_t score;          // last combined S (0..~999), for telemetry
} RiskState;

void risk_init(RiskState *st);

// Call whenever shock_update() (features.h) reports a closed event. Returns
// this event's own score (999 if saturated), so the caller can tell a new
// L2+ hazard apart from a minor bump while an alert is already latched.
uint16_t risk_on_shock_event(RiskState *st, const Profile *p, uint16_t peak_mg,
                          uint16_t dv_mms, uint8_t saturated);

// Call once per second. `tilt_x10` is the latest angle from tilt_update()
// and `dwell_s` is TiltTracker.dwell_s. Returns the combined score S.
uint16_t risk_tick_1s(RiskState *st, const Profile *p, int16_t tilt_x10,
                       float dwell_s);

// Hysteresis level tracker (docs/design.md 5.3: drop needs score < enter
// threshold - 10 for 5 consecutive seconds).
typedef struct {
  uint8_t level;
  uint8_t below_count_s;
} LevelTracker;

void level_tracker_init(LevelTracker *lt);
// Call once per second with the latest score; returns the debounced level.
uint8_t level_tracker_update(LevelTracker *lt, uint16_t score);

#ifdef __cplusplus
}
#endif
