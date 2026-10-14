// Pure C, no Arduino.h. Frame format is documented in docs/design.md section 8.
//   [SOF][LEN][TYPE][SEQ][PAYLOAD (LEN bytes)][CRC8]
// SOF is 0xA5 node->PC, 0x5A PC->node. LEN counts only the payload bytes.
// CRC8 (see crc8.h) is computed over LEN, TYPE, SEQ and PAYLOAD.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SG_SOF_TO_PC 0xA5
#define SG_SOF_TO_NODE 0x5A

// Node -> PC message types.
#define SG_TYPE_TELEMETRY 0x01  // periodic summary
#define SG_TYPE_EVENT 0x02      // shock event (same payload as TELEMETRY)
#define SG_TYPE_ALERT 0x03      // level transition (same payload as TELEMETRY)
#define SG_TYPE_ACK_LOG 0x04    // driver acknowledged an alert
#define SG_TYPE_STATUS 0x05     // firmware/runtime status
#define SG_TYPE_PRESENCE 0x06   // a person arrived at the package (same payload as TELEMETRY)

// PC -> node command types.
#define SG_CMD_ACK 0x81
#define SG_CMD_SET_PROFILE 0x82
#define SG_CMD_PING 0x84
#define SG_CMD_SET_INTERVAL 0x85  // 1-byte report period in seconds (1-60)
#define SG_CMD_ARM 0x87           // new shipment: recalibrate tilt, reset risk/level/FSM

// bit flags for TelemetryPayload.flags
#define SG_FLAG_SAT 0x01
#define SG_FLAG_OPEN 0x02
#define SG_FLAG_PIR 0x04
#define SG_FLAG_AIR 0x08
#define SG_FLAG_ACK_PENDING 0x10
#define SG_FLAG_SENSOR_FAULT 0x20

#define SG_PAYLOAD_MAX 24
// SOF + LEN + TYPE + SEQ + payload + CRC
#define SG_FRAME_MAX (5 + SG_PAYLOAD_MAX)

// Shared 24-byte payload for TELEMETRY / EVENT / ALERT (docs/design.md 8.2).
// vib_rms_mg is always 0 in the MVP (reserved for Extension 1). state is
// 0=MONITORING, 1=ALERT in the MVP; values 2-4 are reserved for the
// Extension-1 STORAGE/TRANSIT/HANDLING split so the wire format never changes.
typedef struct {
  uint32_t t_s;
  uint8_t state;
  uint8_t level;
  uint8_t profile;
  uint8_t flags;
  uint16_t risk;
  uint16_t peak_mg;
  uint16_t dv_mms;
  uint16_t vib_rms_mg;
  int16_t tilt_x10;
  int16_t temp_x100;
  uint16_t hum_x100;
  uint16_t press_x10;
} TelemetryPayload;

// Packs `p` into a 24-byte little-endian buffer.
void telemetry_pack(const TelemetryPayload *p, uint8_t out[SG_PAYLOAD_MAX]);

// Builds a complete outbound frame into `out` (must be >= SG_FRAME_MAX bytes).
// Returns the number of bytes written, or 0 if payload_len > SG_PAYLOAD_MAX.
uint8_t frame_build(uint8_t type, uint8_t seq, const uint8_t *payload,
                     uint8_t payload_len, uint8_t *out);

// Incremental parser for inbound PC -> node frames. Feed one byte at a time
// (e.g. from Serial.read()); this never blocks and never allocates.
typedef struct {
  uint8_t state;
  uint8_t len;
  uint8_t type;
  uint8_t seq;
  uint8_t payload[SG_PAYLOAD_MAX];
  uint8_t payload_pos;
} FrameParser;

typedef struct {
  uint8_t type;
  uint8_t payload[SG_PAYLOAD_MAX];
  uint8_t payload_len;
} ParsedFrame;

void frame_parser_init(FrameParser *fp);
// Returns 1 when `out` now holds a complete, CRC-valid frame; 0 otherwise.
// A bad SOF, an over-length LEN, or a CRC mismatch silently resets the parser
// (the caller does not need to distinguish "no frame yet" from "bad frame").
uint8_t frame_parser_feed(FrameParser *fp, uint8_t byte, ParsedFrame *out);

// Returns 1 while the parser is waiting for a start byte (not inside a frame).
// The sketch uses this so a debug text command never steals a frame byte.
uint8_t frame_parser_idle(const FrameParser *fp);

#ifdef __cplusplus
}
#endif
