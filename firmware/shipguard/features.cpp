#include "features.h"
#include <math.h>

// 32-bit integer square root (binary search). Used instead of sqrtf() on the
// 400 Hz path to avoid software floating point in the tightest loop.
static uint32_t isqrt32(uint32_t v) {
  uint32_t res = 0;
  uint32_t bit = 1UL << 30;  // highest power of 4 <= UINT32_MAX
  while (bit > v) bit >>= 2;
  while (bit != 0) {
    if (v >= res + bit) {
      v -= res + bit;
      res = (res >> 1) + bit;
    } else {
      res >>= 1;
    }
    bit >>= 2;
  }
  return res;
}

uint16_t accel_magnitude_mg(const AccelMg *s) {
  int32_t x = s->x, y = s->y, z = s->z;
  uint32_t sq = (uint32_t)(x * x) + (uint32_t)(y * y) + (uint32_t)(z * z);
  uint32_t mag = isqrt32(sq);
  return (mag > 0xFFFF) ? 0xFFFF : (uint16_t)mag;
}

static uint8_t axis_saturated(int16_t v) {
  int16_t a = (v < 0) ? (int16_t)(-v) : v;
  return a >= (int16_t)SHOCK_SATURATION_MG;
}

void shock_init(ShockDetector *sd) {
  sd->active = 0;
  sd->peak_mg = 0;
  sd->dyn_sum = 0;
  sd->sample_count = 0;
  sd->quiet_count = 0;
  sd->saturated = 0;
}

ShockEvent shock_update(ShockDetector *sd, const AccelMg *sample) {
  ShockEvent out;
  out.event_ended = 0;
  out.peak_mg = 0;
  out.dv_mms = 0;
  out.saturated = 0;

  uint16_t mag = accel_magnitude_mg(sample);
  // dyn = |mag - 1000| (gravity removed). Both operands are small enough
  // that plain int32 arithmetic is enough headroom.
  int32_t dyn32 = (int32_t)mag - 1000;
  uint16_t dyn = (uint16_t)((dyn32 < 0) ? -dyn32 : dyn32);
  uint8_t sat_now = axis_saturated(sample->x) || axis_saturated(sample->y) ||
                     axis_saturated(sample->z);

  if (!sd->active) {
    if (dyn > SHOCK_START_MG) {
      sd->active = 1;
      sd->peak_mg = dyn;
      sd->dyn_sum = dyn;
      sd->sample_count = 1;
      sd->quiet_count = 0;
      sd->saturated = sat_now;
    }
    return out;
  }

  // Event in progress.
  if (dyn > sd->peak_mg) sd->peak_mg = dyn;
  sd->dyn_sum += dyn;
  sd->sample_count++;
  sd->saturated = sd->saturated || sat_now;
  sd->quiet_count = (dyn < SHOCK_END_MG) ? (uint8_t)(sd->quiet_count + 1) : 0;

  uint8_t should_close = (sd->quiet_count >= SHOCK_END_QUIET_SAMPLES) ||
                          (sd->sample_count >= SHOCK_MAX_SAMPLES);
  if (should_close) {
    out.event_ended = 1;
    out.peak_mg = sd->peak_mg;
    // 1 mg held for one 2.5 ms sample contributes ~0.0245 mm/s of delta-V;
    // 1/0.0245 ~= 41. See docs/design.md section 4 for the derivation. This
    // is a magnitude-only approximation (direction is discarded), and for a
    // saturated event it is a lower bound, since clipped samples read low.
    out.dv_mms = (uint16_t)(sd->dyn_sum / 41);
    out.saturated = sd->saturated;
    shock_init(sd);
  }
  return out;
}

void tilt_init(TiltTracker *tt) {
  tt->ref_x = 0;
  tt->ref_y = 0;
  tt->ref_z = 1;
  tt->calibrated = 0;
  tt->dwell_s = 0;
  tt->last_angle_x10 = 0;
}

void tilt_calibrate(TiltTracker *tt, const AccelMg *sample) {
  uint16_t mag = accel_magnitude_mg(sample);
  if (mag == 0) return;  // refuse a bogus all-zero calibration sample
  float inv = 1.0f / (float)mag;
  tt->ref_x = sample->x * inv;
  tt->ref_y = sample->y * inv;
  tt->ref_z = sample->z * inv;
  tt->calibrated = 1;
  tt->dwell_s = 0;
}

int16_t tilt_update(TiltTracker *tt, const AccelMg *sample, float dt_s,
                     int16_t tilt_limit_x10) {
  if (!tt->calibrated) return 0;

  uint16_t mag = accel_magnitude_mg(sample);
  int32_t dyn32 = (int32_t)mag - 1000;
  uint16_t dyn = (uint16_t)((dyn32 < 0) ? -dyn32 : dyn32);
  if (dyn >= TILT_STATIC_GATE_MG || mag == 0) {
    // Board is moving too much to trust this reading; freeze angle and dwell
    // (docs/design.md section 4, "정적 판정 게이트").
    return tt->last_angle_x10;
  }

  float inv = 1.0f / (float)mag;
  float nx = sample->x * inv, ny = sample->y * inv, nz = sample->z * inv;
  float dot = nx * tt->ref_x + ny * tt->ref_y + nz * tt->ref_z;
  if (dot > 1.0f) dot = 1.0f;
  if (dot < -1.0f) dot = -1.0f;
  float angle_deg = acosf(dot) * (180.0f / (float)M_PI);
  int16_t angle_x10 = (int16_t)(angle_deg * 10.0f + 0.5f);

  // 5 deg deadband: resting jitter measured at +-3.5 deg (smoke test 2026-09-21).
  int16_t effective = (angle_x10 < TILT_DEADBAND_X10) ? 0 : angle_x10;
  if (effective > tilt_limit_x10) {
    tt->dwell_s += dt_s;
  } else {
    tt->dwell_s = 0;
  }
  tt->last_angle_x10 = angle_x10;
  return angle_x10;
}
