#include "v2x_codec.h"
#include <string.h>

static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void wr32(uint8_t *p, uint32_t v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }
static void wr64(uint8_t *p, uint64_t v) { wr32(p, (uint32_t)v); wr32(p+4, (uint32_t)(v>>32)); }
static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }
static uint64_t rd64(const uint8_t *p) { return (uint64_t)rd32(p) | ((uint64_t)rd32(p+4)<<32); }

size_t v2x_encode(const v2x_state_t *s, uint8_t *buf, size_t len)
{
    if (len < V2X_WIRE_SIZE) return 0;
    memset(buf, 0, V2X_WIRE_SIZE);
    wr32(&buf[0], s->vehicle_id);
    wr64(&buf[4], s->timestamp_us);
    memcpy(&buf[12], &s->px, 4);
    memcpy(&buf[16], &s->py, 4);
    memcpy(&buf[20], &s->vx, 4);
    memcpy(&buf[24], &s->vy, 4);
    buf[28] = s->intent;
    const uint16_t crc = crc16(buf, V2X_WIRE_SIZE - 2);
    buf[30] = (uint8_t)crc; buf[31] = (uint8_t)(crc >> 8);
    return V2X_WIRE_SIZE;
}

bool v2x_decode(const uint8_t *buf, size_t len, v2x_state_t *out)
{
    if (len < V2X_WIRE_SIZE) return false;
    const uint16_t rx = (uint16_t)(buf[30] | (buf[31] << 8));
    if (crc16(buf, V2X_WIRE_SIZE - 2) != rx) return false;
    out->vehicle_id  = rd32(&buf[0]);
    out->timestamp_us = rd64(&buf[4]);
    memcpy(&out->px, &buf[12], 4);
    memcpy(&out->py, &buf[16], 4);
    memcpy(&out->vx, &buf[20], 4);
    memcpy(&out->vy, &buf[24], 4);
    out->intent = buf[28];
    return out->intent <= INTENT_STOP;
}
