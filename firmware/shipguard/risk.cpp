#include "risk.h"

// Example profiles, not certified values (docs/design.md 5.1). Calibrate
// against real box-drop tests before trusting these for anything but a demo.
const Profile PROFILES[PROFILE_COUNT] = {
    /* name          g_crit  dv_crit  tilt_limit_x10  tilt_dwell_s */
    {"GLASS", 4000, 1500, 300, 30},
    {"ELECTRONICS", 8000, 2500, 450, 60},
    {"ROBUST", 14000, 4000, 600, 120},
};

void risk_init(RiskState *st) {
  st->shock_score = 0;
  st->shock_hold_s = 0;
  st->shock_saturated = 0;
  st->score = 0;
}

uint16_t risk_on_shock_event(RiskState *st, const Profile *p, uint16_t peak_mg,
                          uint16_t dv_mms, uint8_t saturated) {
  uint16_t r1 = (uint16_t)(((uint32_t)peak_mg * 100) / p->g_crit_mg);
  uint16_t r2 = (uint16_t)(((uint32_t)dv_mms * 100) / p->dv_crit_mms);
  uint16_t r = (r1 > r2) ? r1 : r2;
  // A saturated accelerometer reading means the true peak is unknown but at
  // least the sensor's full scale, so we do not trust r1/r2 and force L3
  // instead of silently under-reporting (docs/design.md 5.2 and 11.2).
  if (saturated) r = 999;
  if (r > st->shock_score) {
    st->shock_score = r;
    st->shock_hold_s = 30;
    st->shock_saturated = saturated;
  }
  return r;
}

uint16_t risk_tick_1s(RiskState *st, const Profile *p, int16_t tilt_x10,
                       float dwell_s) {
  if (st->shock_hold_s > 0) {
    st->shock_hold_s--;
  } else if (st->shock_score > 0) {
    st->shock_score = (st->shock_score > 2) ? (uint16_t)(st->shock_score - 2) : 0;
  }

  uint16_t r_tilt = (uint16_t)((100.0f * dwell_s) / (float)p->tilt_dwell_s);
  if (r_tilt > 999) r_tilt = 999;
  int16_t limit_x1_5 = (int16_t)(((int32_t)p->tilt_limit_x10 * 3) / 2);
  if (tilt_x10 > limit_x1_5 && r_tilt < 70) r_tilt = 70;

  st->score = (st->shock_score > r_tilt) ? st->shock_score : r_tilt;
  return st->score;
}

void level_tracker_init(LevelTracker *lt) {
  lt->level = RISK_LEVEL_L0;
  lt->below_count_s = 0;
}

static uint8_t raw_level(uint16_t score) {
  if (score >= 100) return RISK_LEVEL_L3;
  if (score >= 70) return RISK_LEVEL_L2;
  if (score >= 40) return RISK_LEVEL_L1;
  return RISK_LEVEL_L0;
}

// Enter thresholds are 40/70/100; a drop needs the score under
// (enter threshold - 10) for 5 consecutive seconds (docs/design.md 5.3).
static uint16_t drop_threshold_for(uint8_t level) {
  switch (level) {
    case RISK_LEVEL_L3: return 90;
    case RISK_LEVEL_L2: return 60;
    case RISK_LEVEL_L1: return 30;
    default: return 0;
  }
}

uint8_t level_tracker_update(LevelTracker *lt, uint16_t score) {
  uint8_t raw = raw_level(score);
  if (raw > lt->level) {
    lt->level = raw;
    lt->below_count_s = 0;
    return lt->level;
  }
  if (raw < lt->level) {
    if (score < drop_threshold_for(lt->level)) {
      lt->below_count_s++;
      if (lt->below_count_s >= 5) {
        lt->level = raw;
        lt->below_count_s = 0;
      }
    } else {
      lt->below_count_s = 0;
    }
    return lt->level;
  }
  lt->below_count_s = 0;
  return lt->level;
}
