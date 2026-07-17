/* Seven documented test cases with concrete inputs and expected results.
 * Mirrors docs/TEST_CASES.md. */
#include "unity.h"
#include <string.h>
#include "kalman.h"
#include "v2x_codec.h"
#include "brake_controller.h"

void setUp(void) {}
void tearDown(void) {}

static v2x_state_t sample(void)
{
    v2x_state_t s = { .vehicle_id = 42, .timestamp_us = 123456789ULL,
                      .px = 10.5f, .py = -3.25f, .vx = 8.0f, .vy = 0.5f,
                      .intent = INTENT_LEFT };
    return s;
}

/* TC-1  Kalman speed convergence: 50 updates of z=10 m/s -> 9.999 m/s */
void test_tc1_kf_speed_converges_to_measurement(void)
{
    kf_t kf;
    kf_init(&kf, 0, 0, 0.5f, 1.5f, 0.2f);
    for (int i = 0; i < 50; i++) { kf_predict(&kf, 0.02f); kf_update_speed(&kf, 10.0f, 0.0f); }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, kf_speed(&kf));
}

/* TC-2  Mahalanobis gate: after settling at origin, GPS fix at (1000,1000)
 *       with 3-sigma gate is rejected (returns false) */
void test_tc2_kf_gps_outlier_rejected(void)
{
    kf_t kf;
    kf_init(&kf, 0, 0, 0.5f, 1.5f, 0.2f);
    for (int i = 0; i < 20; i++) { kf_predict(&kf, 0.02f); kf_update_gps(&kf, 0.0f, 0.0f, 3.0f); }
    TEST_ASSERT_TRUE(kf_update_gps(&kf, 0.1f, 0.1f, 3.0f));
    TEST_ASSERT_FALSE(kf_update_gps(&kf, 1000.0f, 1000.0f, 3.0f));
}

/* TC-3  v2x_encode/v2x_decode roundtrip: id 42, px 10.5, intent LEFT survive
 *       the 32-byte wire format bit-exact */
void test_tc3_v2x_roundtrip(void)
{
    uint8_t buf[V2X_WIRE_SIZE];
    v2x_state_t in = sample(), out;
    TEST_ASSERT_EQUAL_size_t(V2X_WIRE_SIZE, v2x_encode(&in, buf, sizeof(buf)));
    TEST_ASSERT_TRUE(v2x_decode(buf, sizeof(buf), &out));
    TEST_ASSERT_EQUAL_UINT32(42, out.vehicle_id);
    TEST_ASSERT_EQUAL_UINT64(123456789ULL, out.timestamp_us);
    TEST_ASSERT_EQUAL_FLOAT(10.5f, out.px);
    TEST_ASSERT_EQUAL_UINT8(INTENT_LEFT, out.intent);
}

/* TC-4  Corrupted byte -> CRC mismatch -> decode returns false */
void test_tc4_v2x_corruption_detected(void)
{
    uint8_t buf[V2X_WIRE_SIZE];
    v2x_state_t in = sample(), out;
    v2x_encode(&in, buf, sizeof(buf));
    buf[10] ^= 0xFF;
    TEST_ASSERT_FALSE(v2x_decode(buf, sizeof(buf), &out));
}

/* TC-5  Invalid intent enum (7) on the wire -> decode returns false */
void test_tc5_v2x_invalid_intent_rejected(void)
{
    uint8_t buf[V2X_WIRE_SIZE];
    v2x_state_t in = sample(), out;
    in.intent = 7;
    v2x_encode(&in, buf, sizeof(buf));
    TEST_ASSERT_FALSE(v2x_decode(buf, sizeof(buf), &out));
}

/* TC-6  Brake ramp: rate 2.0/s. 100 ms after emergency -> 0.200 (RAMP);
 *       600 ms -> 1.000 (FULL), engage_count 1 */
void test_tc6_brake_ramp_profile(void)
{
    brake_ctrl_t b;
    brake_init(&b, 2.0f);
    brake_emergency(&b);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2f, brake_step(&b, 0.1f));
    TEST_ASSERT_EQUAL(BRAKE_RAMP, b.state);
    float cmd = 0.0f;
    for (int i = 0; i < 5; i++) cmd = brake_step(&b, 0.1f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, cmd);
    TEST_ASSERT_EQUAL(BRAKE_FULL, b.state);
    TEST_ASSERT_EQUAL_UINT32(1, b.engage_count);
}

/* TC-7  Release from FULL ramps back to 0.000 and returns to IDLE */
void test_tc7_brake_release_returns_to_idle(void)
{
    brake_ctrl_t b;
    brake_init(&b, 2.0f);
    brake_emergency(&b);
    for (int i = 0; i < 6; i++) brake_step(&b, 0.1f);
    brake_release(&b);
    float cmd = 1.0f;
    for (int i = 0; i < 6; i++) cmd = brake_step(&b, 0.1f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd);
    TEST_ASSERT_EQUAL(BRAKE_IDLE, b.state);
}
