#ifndef V2X_CODEC_H
#define V2X_CODEC_H
/* Compact V2X state message (wire: 32 bytes, little-endian, CRC-16 trailer). */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef enum { INTENT_STRAIGHT = 0, INTENT_LEFT = 1, INTENT_RIGHT = 2, INTENT_STOP = 3 } v2x_intent_t;

typedef struct {
    uint32_t vehicle_id;
    uint64_t timestamp_us;   /* EtherCAT DC-synchronized */
    float    px, py;         /* m, local intersection frame */
    float    vx, vy;         /* m/s */
    uint8_t  intent;         /* v2x_intent_t */
} v2x_state_t;

#define V2X_WIRE_SIZE 32u

size_t v2x_encode(const v2x_state_t *s, uint8_t *buf, size_t len);
bool   v2x_decode(const uint8_t *buf, size_t len, v2x_state_t *out);

#endif
