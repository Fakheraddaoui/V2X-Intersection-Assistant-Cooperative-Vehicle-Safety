#include "unity.h"
#include "v2x_codec.h"
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

static v2x_state_t sample(void)
{
    v2x_state_t s = {
        .vehicle_id = 42, .timestamp_us = 1234567890123ULL,
        .px = 10.5f, .py = -3.25f, .vx = 8.0f, .vy = 0.5f,
        .intent = INTENT_LEFT
    };
    return s;
}

void test_encode_decode_roundtrip(void)
{
    uint8_t buf[V2X_WIRE_SIZE];
    v2x_state_t in = sample(), out;
    TEST_ASSERT_EQUAL_size_t(V2X_WIRE_SIZE, v2x_encode(&in, buf, sizeof(buf)));
    TEST_ASSERT_TRUE(v2x_decode(buf, sizeof(buf), &out));
    TEST_ASSERT_EQUAL_UINT32(42, out.vehicle_id);
    TEST_ASSERT_EQUAL_UINT64(1234567890123ULL, out.timestamp_us);
    TEST_ASSERT_EQUAL_FLOAT(10.5f, out.px);
    TEST_ASSERT_EQUAL_FLOAT(-3.25f, out.py);
    TEST_ASSERT_EQUAL_UINT8(INTENT_LEFT, out.intent);
}

void test_decode_rejects_bad_crc(void)
{
    uint8_t buf[V2X_WIRE_SIZE];
    v2x_state_t in = sample(), out;
    v2x_encode(&in, buf, sizeof(buf));
    buf[12] ^= 0x01;
    TEST_ASSERT_FALSE(v2x_decode(buf, sizeof(buf), &out));
}

void test_decode_rejects_invalid_intent(void)
{
    uint8_t buf[V2X_WIRE_SIZE];
    v2x_state_t in = sample(), out;
    in.intent = 99;
    v2x_encode(&in, buf, sizeof(buf));
    TEST_ASSERT_FALSE(v2x_decode(buf, sizeof(buf), &out));
}

void test_encode_fails_on_small_buffer(void)
{
    uint8_t buf[8];
    v2x_state_t in = sample();
    TEST_ASSERT_EQUAL_size_t(0, v2x_encode(&in, buf, sizeof(buf)));
}
