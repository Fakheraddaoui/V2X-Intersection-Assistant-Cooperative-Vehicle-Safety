#ifndef BRAKE_CONTROLLER_H
#define BRAKE_CONTROLLER_H
#include <stdint.h>
#include <stdbool.h>

typedef enum { BRAKE_IDLE, BRAKE_RAMP, BRAKE_FULL, BRAKE_RELEASE } brake_state_t;

typedef struct {
    brake_state_t state;
    float command;        /* 0..1 brake pressure fraction */
    float ramp_rate;      /* fraction per second */
    uint32_t engage_count;
} brake_ctrl_t;

void  brake_init(brake_ctrl_t *b, float ramp_rate);
void  brake_emergency(brake_ctrl_t *b);
void  brake_release(brake_ctrl_t *b);
float brake_step(brake_ctrl_t *b, float dt);   /* call at EtherCAT cycle rate */

#endif
