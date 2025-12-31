#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern volatile float external_setpoint_rpm;

void external_setpoint_init(void);

#ifdef __cplusplus
}
#endif
