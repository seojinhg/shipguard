#include "frame.h"
#include "crc8.h"

static void put_u16(uint8_t *out, uint16_t v) {
  out[0] = (uint8_t)(v & 0xFF);
  out[1] = (uint8_t)((v >> 8) & 0xFF);
}
static void put_u32(uint8_t *out, uint32_t v) {
  out[0] = (uint8_t)(v & 0xFF);
  out[1] = (uint8_t)((v >> 8) & 0xFF);
  out[2] = (uint8_t)((v >> 16) & 0xFF);
  out[3] = (uint8_t)((v >> 24) & 0xFF);
}

void telemetry_pack(const TelemetryPayload *p, uint8_t out[SG_PAYLOAD_MAX]) {
  put_u32(&out[0], p->t_s);
  out[4] = p->state;
  out[5] = p->level;
  out[6] = p->profile;
  out[7] = p->flags;
  put_u16(&out[8], p->risk);
  put_u16(&out[10], p->peak_mg);
  put_u16(&out[12], p->dv_mms);
  put_u16(&out[14], p->vib_rms_mg);
  put_u16(&out[16], (uint16_t)p->tilt_x10);
  put_u16(&out[18], (uint16_t)p->temp_x100);
  put_u16(&out[20], p->hum_x100);
  put_u16(&out[22], p->press_x10);
}

uint8_t frame_build(uint8_t type, uint8_t seq, const uint8_t *payload,
                     uint8_t payload_len, uint8_t *out) {
  if (payload_len > SG_PAYLOAD_MAX) return 0;
  out[0] = SG_SOF_TO_PC;
  out[1] = payload_len;
  out[2] = type;
  out[3] = seq;
  for (uint8_t i = 0; i < payload_len; i++) out[4 + i] = payload[i];
  // CRC covers LEN, TYPE, SEQ and PAYLOAD (i.e. everything after SOF).
  uint8_t crc = crc8_buffer(&out[1], (uint16_t)(3 + payload_len));
  out[4 + payload_len] = crc;
  return (uint8_t)(5 + payload_len);
}

// Parser states.
enum { PS_SOF = 0, PS_LEN, PS_TYPE, PS_SEQ, PS_PAYLOAD, PS_CRC };

uint8_t frame_parser_idle(const FrameParser *fp) { return fp->state == PS_SOF; }

void frame_parser_init(FrameParser *fp) {
  fp->state = PS_SOF;
  fp->len = 0;
  fp->type = 0;
  fp->seq = 0;
  fp->payload_pos = 0;
}

uint8_t frame_parser_feed(FrameParser *fp, uint8_t byte, ParsedFrame *out) {
  switch (fp->state) {
    case PS_SOF:
      if (byte == SG_SOF_TO_NODE) fp->state = PS_LEN;
      // else: stay in PS_SOF, ignore noise until we see a real SOF.
      return 0;
    case PS_LEN:
      if (byte > SG_PAYLOAD_MAX) {  // malformed length, resync
        fp->state = PS_SOF;
        return 0;
      }
      fp->len = byte;
      fp->state = PS_TYPE;
      return 0;
    case PS_TYPE:
      fp->type = byte;
      fp->state = PS_SEQ;
      return 0;
    case PS_SEQ:
      fp->seq = byte;
      fp->payload_pos = 0;
      fp->state = (fp->len == 0) ? PS_CRC : PS_PAYLOAD;
      return 0;
    case PS_PAYLOAD:
      fp->payload[fp->payload_pos++] = byte;
      if (fp->payload_pos >= fp->len) fp->state = PS_CRC;
      return 0;
    case PS_CRC: {
      // Rebuild the same byte order frame_build() used (LEN, TYPE, SEQ,
      // PAYLOAD) to recompute the CRC before checking it.
      uint8_t tmp[3 + SG_PAYLOAD_MAX];
      tmp[0] = fp->len;
      tmp[1] = fp->type;
      tmp[2] = fp->seq;
      for (uint8_t i = 0; i < fp->len; i++) tmp[3 + i] = fp->payload[i];
      uint8_t expected = crc8_buffer(tmp, (uint16_t)(3 + fp->len));
      fp->state = PS_SOF;  // ready for the next frame either way
      if (byte != expected) return 0;
      out->type = fp->type;
      out->payload_len = fp->len;
      for (uint8_t i = 0; i < fp->len; i++) out->payload[i] = fp->payload[i];
      return 1;
    }
    default:
      fp->state = PS_SOF;
      return 0;
  }
}
