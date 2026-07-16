#include "brake_controller.h"

void brake_init(brake_ctrl_t *b, float ramp_rate)
{
    b->state = BRAKE_IDLE;
    b->command = 0.0f;
    b->ramp_rate = ramp_rate;
    b->engage_count = 0;
}

void brake_emergency(brake_ctrl_t *b)
{
    if (b->state == BRAKE_IDLE || b->state == BRAKE_RELEASE) {
        b->state = BRAKE_RAMP;
        b->engage_count++;
    }
}

void brake_release(brake_ctrl_t *b)
{
    if (b->state == BRAKE_FULL || b->state == BRAKE_RAMP)
        b->state = BRAKE_RELEASE;
}

float brake_step(brake_ctrl_t *b, float dt)
{
    switch (b->state) {
    case BRAKE_RAMP:
        b->command += b->ramp_rate * dt;
        if (b->command >= 1.0f) { b->command = 1.0f; b->state = BRAKE_FULL; }
        break;
    case BRAKE_RELEASE:
        b->command -= b->ramp_rate * dt;
        if (b->command <= 0.0f) { b->command = 0.0f; b->state = BRAKE_IDLE; }
        break;
    default:
        break;
    }
    return b->command;
}
