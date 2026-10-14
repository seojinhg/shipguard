#include "crc8.h"

// Bit-by-bit CRC-8, poly 0x07 (x^8 + x^2 + x + 1), no reflection, no final XOR.
// Not the fastest form (no lookup table), but the frames are short (<=29 bytes)
// and this keeps flash usage down, which matters more on the ATmega328P.
uint8_t crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t bit = 0; bit < 8; bit++) {
    if (crc & 0x80) {
      crc = (uint8_t)((crc << 1) ^ 0x07);
    } else {
      crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

uint8_t crc8_buffer(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc = crc8_update(crc, data[i]);
  }
  return crc;
}
