#include "setpoint_task.h"
#include <math.h>
#include "esp_timer.h"
#include "freertos/task.h"

// Define the variables here (without 'extern')
volatile float SETPOINT_RPM_SHARED = 0;
portMUX_TYPE setpoint_mux = portMUX_INITIALIZER_UNLOCKED;
setpoint_mode_t current_mode = SETPOINT_MODE_UP_AND_DOWN;
// Constants used only by the setpoint logic
#define MAX_RPM 650.0f
#define MIN_RPM 50.0f
#define RPM_STEP 10.0f
#define SINE_AMPLITUDE 100.0f
#define SINE_PERIOD_MS 2000 
#define SINE_OFFSET_MIN 350.0f
#define SINE_OFFSET_MAX 575.0f
#define OFFSET_STEP_SIZE 50.0f
#define SINE_CYCLES_PER_STEP 20

// Logic moved from main.c
// ===== SETPOINT GENERATION TASK (REVISED FOR RELIABLE SWEEP) =====
void setpoint_task(void *arg)
{
    float rpm = 0.0f; // Start at 0 RPM
    int cycle_complete = 0; 

    // --- State Variables for Step Sweep ---
    float current_step_target = 50; 
    if (current_mode == SETPOINT_MODE_STEP_DOWN) current_step_target = MAX_RPM;
// Starts at 50 RPM
    int hold_count = 0; // Tracks if we're on the first (acceleration) or second (hold) second

    int64_t t0 = esp_timer_get_time(); // For sine wave timing

    TickType_t xLastWakeStep = xTaskGetTickCount();
    TickType_t xLastWakeSine = xTaskGetTickCount();
    const TickType_t STEP_PERIOD = pdMS_TO_TICKS(2000);
    const TickType_t SINE_SAMPLING_PERIOD = pdMS_TO_TICKS(20);

    static float current_sine_offset = SINE_OFFSET_MIN; 
    static int cycle_counter_sine = 0;

    while (1) {
        if (current_mode == SETPOINT_MODE_STEP) {
            
            if (!cycle_complete) {
                
                // --- Sweep Logic ---
                if (hold_count == 0) {
                    // First second: Step to the new target
                    rpm = current_step_target;
                    hold_count = 1;

                } else {
                    // Second second: Hold at the current target and prepare for the next step
                    rpm = current_step_target; // Still holding

                    // Increment target for the next cycle
                    current_step_target += 50;
                    hold_count = 0; // Reset hold counter
                    
                    if (current_step_target > MAX_RPM) {
                        rpm = 0.0f; 
                        cycle_complete = 1; 
                        printf("INFO: Step sweep finished. Setpoint held at 0.\n");
                    }
                }
            } else {
                // Sweep is complete. Hold setpoint at 0.
                rpm = 0.0f;
            }
            
            // --- Shared Variable Update ---
            taskENTER_CRITICAL(&setpoint_mux);
            SETPOINT_RPM_SHARED = rpm;
            taskEXIT_CRITICAL(&setpoint_mux);
            
            vTaskDelayUntil(&xLastWakeStep, STEP_PERIOD);

        } 
        else if (current_mode == SETPOINT_MODE_UP_AND_DOWN) {
            
            if (!cycle_complete) {
                
                // --- Sweep Logic ---
                if (hold_count == 0) {
                    
                    rpm = current_step_target; //  
                    hold_count = 1;

                } else {
                    // Second second: step to new target
                    rpm = 0;

                    // Increment target for the next cycle
                    current_step_target += 50;
                    hold_count = 0; // Reset hold counter
                    
                    if (current_step_target > MAX_RPM) {
                        rpm = 0.0f; 
                        cycle_complete = 1; 
                        printf("INFO: Step sweep finished. Setpoint held at 0.\n");
                    }
                }
            } else {
                // Sweep is complete. Hold setpoint at 0.
                rpm = 0.0f;
            }
            
            // --- Shared Variable Update ---
            taskENTER_CRITICAL(&setpoint_mux);
            SETPOINT_RPM_SHARED = rpm;
            taskEXIT_CRITICAL(&setpoint_mux);
            
            vTaskDelayUntil(&xLastWakeStep, STEP_PERIOD);

        } 
        else if (current_mode == SETPOINT_MODE_STEP_DOWN) {
            if (!cycle_complete) {
                
                // --- Sweep Logic ---
                if (hold_count == 0) {
                    // First second: Step to the new target
                    rpm = current_step_target;
                    hold_count = 1;

                } else {
                    // Second second: Hold at the current target and prepare for the next step
                    rpm = current_step_target; // Still holding

                    current_step_target -= RPM_STEP;
                    hold_count = 0; // Reset hold counter
                    
                    if (current_step_target < MIN_RPM) {
                        rpm = 0.0f; 
                        cycle_complete = 1; 
                        printf("INFO: Step sweep finished. Setpoint held at 0.\n");
                    }
                }
            } else {
                // Sweep is complete. Hold setpoint at 0.
                rpm = 0.0f;
            }
            
            // --- Shared Variable Update ---
            taskENTER_CRITICAL(&setpoint_mux);
            SETPOINT_RPM_SHARED = rpm;
            taskEXIT_CRITICAL(&setpoint_mux);
            
            vTaskDelayUntil(&xLastWakeStep, STEP_PERIOD);

        } 
        
        // --- SINE MODE LOGIC ---
        else if (current_mode == SETPOINT_MODE_SINE) {
            int64_t now = esp_timer_get_time();
                float t = now / 1000000.0f;
                // Rapidly oscillate around a high-speed mean
                rpm = 500.0f + 50.0f * sinf(2 * M_PI * 2.0f * t); // 2Hz oscillation at 500 RPM
                
                taskENTER_CRITICAL(&setpoint_mux);
                SETPOINT_RPM_SHARED = rpm;
                taskEXIT_CRITICAL(&setpoint_mux);
                vTaskDelay(pdMS_TO_TICKS(20));
        }

        else if (current_mode == SETPOINT_MODE_PERIODIC_SETPOINT_SINE){
            int64_t now = esp_timer_get_time();
            float t = (now - t0) / 1000.0f; // ms

            rpm = current_sine_offset + SINE_AMPLITUDE * sinf(2 * M_PI * t / SINE_PERIOD_MS);
            if (rpm < 0) rpm = 0;

            if (++cycle_counter_sine >= (SINE_PERIOD_MS/20)) {
                cycle_counter_sine = 0;

                // Step the center point (Offset)
                current_sine_offset += OFFSET_STEP_SIZE; 

                // If we exceed the max, reset to the min offset and reset sine phase (t0)
                if (current_sine_offset > SINE_OFFSET_MAX) {
                    current_sine_offset = SINE_OFFSET_MIN;
                    t0 = esp_timer_get_time(); // Reset time to start sine phase cleanly
                    printf("INFO: Sine sweep reset to low offset.\n");
                }
            }

            taskENTER_CRITICAL(&setpoint_mux);
            SETPOINT_RPM_SHARED = rpm;
            taskEXIT_CRITICAL(&setpoint_mux);

            vTaskDelayUntil(&xLastWakeSine, SINE_SAMPLING_PERIOD);
        }
    }
}
