// Pure C, no Arduino.h: compiles both in the sketch and on the host (tests/).
// CRC-8, polynomial 0x07, initial value 0x00 (see docs/design.md section 8.1).
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t crc8_update(uint8_t crc, uint8_t data);
uint8_t crc8_buffer(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
