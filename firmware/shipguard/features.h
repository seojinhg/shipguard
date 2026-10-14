// Pure C, no Arduino.h. Shock and tilt feature extraction (docs/design.md
// section 4, MVP subset: no vibration RMS, no cumulative damage).
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// One accelerometer sample in milli-g, gravity included (e.g. resting flat
// gives roughly (0, 0, 1000)).
typedef struct {
  int16_t x, y, z;
} AccelMg;

// sqrt(x*x + y*y + z*z) in mg, computed with a 32-bit integer square root
// (no floating point on the 400 Hz path).
uint16_t accel_magnitude_mg(const AccelMg *s);

// ---------------- Shock / drop event detector (runs at 400 Hz) ----------------
// An event starts when |mag - 1000| exceeds SHOCK_START_MG and ends after
// SHOCK_END_QUIET_SAMPLES consecutive samples below SHOCK_END_MG, or after
// SHOCK_MAX_SAMPLES, whichever comes first.
#define SHOCK_START_MG 2000u
#define SHOCK_END_MG 500u
#define SHOCK_END_QUIET_SAMPLES 8u   // 8 * 2.5 ms = 20 ms of quiet to close
#define SHOCK_MAX_SAMPLES 80u        // 80 * 2.5 ms = 200 ms hard cap
#define SHOCK_SATURATION_MG 15000u   // any axis at/near the +-16 g full scale

typedef struct {
  uint8_t active;
  uint16_t peak_mg;      // largest |dyn| seen so far in this event
  uint32_t dyn_sum;      // running sum of |dyn| samples, for the dV estimate
  uint16_t sample_count;
  uint8_t quiet_count;
  uint8_t saturated;
} ShockDetector;

typedef struct {
  uint8_t event_ended;  // 1 only on the sample where an event just closed
  uint16_t peak_mg;
  uint16_t dv_mms;       // delta-V estimate in mm/s (see docs/design.md 4)
  uint8_t saturated;
} ShockEvent;

void shock_init(ShockDetector *sd);
// Call once per 400 Hz sample. `out` is only meaningful when the return
// value is 1 (an event just ended this sample).
ShockEvent shock_update(ShockDetector *sd, const AccelMg *sample);

// ---------------- Tilt (runs on the ~1 Hz "second" tick) ----------------
// Angle is only trusted while the sensor is near-static (gated), because a
// moving board reads a mix of gravity and linear acceleration (see the
// 2026-09-21 smoke test notes in docs/design.md section 1).
#define TILT_STATIC_GATE_MG 150     // |mag-1000| must be below this to trust the reading
#define TILT_DEADBAND_X10 50        // 5.0 degrees; resting jitter measured at ~3.5 deg

typedef struct {
  float ref_x, ref_y, ref_z;  // unit gravity vector captured at calibration (ARM)
  uint8_t calibrated;
  float dwell_s;               // seconds the angle has exceeded tilt_limit
  int16_t last_angle_x10;      // held over while the static gate is failing
} TiltTracker;

void tilt_init(TiltTracker *tt);
// Captures the current sample as "level" (call this on the ARM command).
void tilt_calibrate(TiltTracker *tt, const AccelMg *sample);
// Updates dwell_s against `tilt_limit_x10` (degrees * 10) using dt_s seconds
// elapsed since the last call, and returns the current tilt angle (degrees
// * 10). Returns the last known angle unchanged when the static gate fails
// or before calibration.
int16_t tilt_update(TiltTracker *tt, const AccelMg *sample, float dt_s,
                     int16_t tilt_limit_x10);

#ifdef __cplusplus
}
#endif
