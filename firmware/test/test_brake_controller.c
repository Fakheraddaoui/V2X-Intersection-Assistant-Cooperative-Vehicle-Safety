#include "unity.h"
#include "brake_controller.h"

static brake_ctrl_t b;

void setUp(void)    { brake_init(&b, 2.0f); } /* full brake in 0.5 s */
void tearDown(void) {}

void test_idle_outputs_zero(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, brake_step(&b, 0.01f));
}

void test_emergency_ramps_to_full(void)
{
    brake_emergency(&b);
    float cmd = 0.0f;
    for (int i = 0; i < 60; i++) cmd = brake_step(&b, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, cmd);
    TEST_ASSERT_EQUAL(BRAKE_FULL, b.state);
}

void test_ramp_is_monotonic(void)
{
    brake_emergency(&b);
    float prev = 0.0f;
    for (int i = 0; i < 50; i++) {
        float c = brake_step(&b, 0.01f);
        TEST_ASSERT_TRUE(c >= prev);
        prev = c;
    }
}

void test_release_returns_to_idle(void)
{
    brake_emergency(&b);
    for (int i = 0; i < 60; i++) brake_step(&b, 0.01f);
    brake_release(&b);
    for (int i = 0; i < 60; i++) brake_step(&b, 0.01f);
    TEST_ASSERT_EQUAL(BRAKE_IDLE, b.state);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, b.command);
}

void test_engage_count_increments_once_per_event(void)
{
    brake_emergency(&b);
    brake_emergency(&b); /* already ramping: ignored */
    TEST_ASSERT_EQUAL_UINT32(1, b.engage_count);
}
