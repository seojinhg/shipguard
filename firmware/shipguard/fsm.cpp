#include "fsm.h"
#include "risk.h"  // RISK_LEVEL_L2

void fsm_init(FsmState *st) {
  st->state = SG_STATE_MONITORING;
  st->acked = 0;
  st->latched = 0;
}

void fsm_tick(FsmState *st, uint8_t level, uint8_t shock_driven) {
  if (level >= RISK_LEVEL_L2) {
    if (st->state != SG_STATE_ALERT) {
      st->state = SG_STATE_ALERT;
      st->acked = 0;
      st->latched = shock_driven ? 1 : 0;
    } else if (shock_driven) {
      st->latched = 1;
    }
    return;
  }
  if (st->state != SG_STATE_ALERT) return;
  // Below L2: an acknowledged alert closes; an unacknowledged tilt-only alert
  // closes by itself once the level is all the way back at L0.
  if (st->acked || (!st->latched && level == RISK_LEVEL_L0)) {
    st->state = SG_STATE_MONITORING;
    st->acked = 0;
    st->latched = 0;
  }
}

uint8_t fsm_ack(FsmState *st) {
  if (st->state != SG_STATE_ALERT || st->acked) return 0;
  st->acked = 1;
  return 1;
}

void fsm_rearm(FsmState *st) {
  if (st->state == SG_STATE_ALERT) st->acked = 0;
}
