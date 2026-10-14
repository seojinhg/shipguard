// Pure C, no Arduino.h. MVP context state machine: MONITORING/ALERT only
// (docs/design.md section 6.1). STORAGE/TRANSIT/HANDLING are Extension 1.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SG_STATE_MONITORING 0
#define SG_STATE_ALERT 1

typedef struct {
  uint8_t state;
  uint8_t acked;  // set by an ACK during ALERT: silences the buzzer, and the
                  // alert clears itself as soon as the level drops below L2
                  // without a second button press. Cleared on every new
                  // ALERT entry and by fsm_rearm().
  uint8_t latched;  // 1 if a shock drove this alert to L2+. A shock is an event
                    // (damage may already have happened), so its alert stays
                    // until ACK. A tilt-only alert is a condition: if the level
                    // falls back to L0 before any ACK it clears by itself.
} FsmState;

void fsm_init(FsmState *st);
// Call once per second with the debounced level from level_tracker_update().
// `shock_driven` is 1 when the shock part of the score is at L2+ (the caller
// passes RiskState.shock_score >= 70).
void fsm_tick(FsmState *st, uint8_t level, uint8_t shock_driven);
// Call when the driver presses the ACK button (or the PC sends SG_CMD_ACK).
// Returns 1 if the ACK was accepted (an unacknowledged ALERT was active), 0 if
// it was ignored, so a stray press in MONITORING cannot pre-silence the next
// alert.
uint8_t fsm_ack(FsmState *st);
// Call when a new hazard arrives while an acknowledged ALERT is still active
// (a new L2+ shock, or the level rising): the alarm sounds again and needs a
// fresh ACK.
void fsm_rearm(FsmState *st);

#ifdef __cplusplus
}
#endif
