#include "unity.h"
#include "kalman.h"
#include <math.h>

static kf_t kf;

void setUp(void)    { kf_init(&kf, 0.0f, 0.0f, 0.5f, 1.5f, 0.2f); }
void tearDown(void) {}

void test_predict_propagates_position_with_velocity(void)
{
    kf.x[2] = 2.0f; kf.x[3] = -1.0f;
    kf_predict(&kf, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, kf.x[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f, kf.x[1]);
}

void test_predict_grows_covariance(void)
{
    const float p0 = kf.P[0][0];
    kf_predict(&kf, 0.1f);
    TEST_ASSERT_TRUE(kf.P[0][0] > p0);
}

void test_gps_update_pulls_state_toward_measurement(void)
{
    TEST_ASSERT_TRUE(kf_update_gps(&kf, 5.0f, 0.0f, 0.0f));
    TEST_ASSERT_TRUE(kf.x[0] > 0.0f && kf.x[0] < 5.0f);
}

void test_gps_update_shrinks_covariance(void)
{
    const float p0 = kf.P[0][0];
    kf_update_gps(&kf, 1.0f, 1.0f, 0.0f);
    TEST_ASSERT_TRUE(kf.P[0][0] < p0);
}

void test_outlier_gps_rejected_by_gate(void)
{
    /* Shrink covariance first so the gate is meaningful */
    for (int i = 0; i < 20; i++) { kf_predict(&kf, 0.05f); kf_update_gps(&kf, 0.0f, 0.0f, 0.0f); }
    TEST_ASSERT_FALSE(kf_update_gps(&kf, 500.0f, 500.0f, 3.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, kf.x[0]); /* state unchanged */
}

void test_speed_update_sets_velocity_along_heading(void)
{
    for (int i = 0; i < 50; i++) kf_update_speed(&kf, 10.0f, 0.0f); /* east */
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 10.0f, kf.x[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, kf.x[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 10.0f, kf_speed(&kf));
}

void test_convergence_on_moving_target(void)
{
    /* Truth: start (0,0), v=(3,0). Simulate 5 s @ 20 Hz of noisy-free GPS. */
    float t = 0.0f;
    for (int i = 0; i < 100; i++) {
        kf_predict(&kf, 0.05f);
        t += 0.05f;
        kf_update_gps(&kf, 3.0f * t, 0.0f, 0.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 3.0f * t, kf.x[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 3.0f, kf.x[2]);
}
