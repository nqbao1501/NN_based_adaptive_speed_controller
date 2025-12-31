#ifndef SETPOINT_TASK_H
#define SETPOINT_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define the enum here so both files know the modes
typedef enum {
    SETPOINT_MODE_STEP = 0,
    SETPOINT_MODE_SINE = 1,
    SETPOINT_MODE_PERIODIC_SETPOINT_SINE = 2,
    SETPOINT_MODE_STEP_DOWN = 3,
    SETPOINT_MODE_UP_AND_DOWN = 4,
    SETPOINT_MODE_EXTERNAL = 5
} setpoint_mode_t;


extern volatile float SETPOINT_RPM_SHARED;
extern portMUX_TYPE setpoint_mux;
extern setpoint_mode_t current_mode;

// Function prototype for the task
void setpoint_task(void *arg);

#endif