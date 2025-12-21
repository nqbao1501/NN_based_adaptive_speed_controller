#include <stdio.h>
#include <math.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/uhci.h"
#include "esp_task_wdt.h"
#include "driver/pulse_cnt.h"
#include "nn_controller.h"
#include "setpoint_task.h"


//Control mode
#define MODE_PI_CONTROLLER      0
#define MODE_COMBINE            2
#define CONTROL_MODE            MODE_COMBINE

//Adaptive gain
float alpha = 1.0f;
const float alpha_min = 0.67f;
const float alpha_max = 1.33f;
const float GAMMA = 0.0000001f; 

// ===== Motor Pins =====
#define MOTOR_IN1_GPIO 35
#define MOTOR_IN2_GPIO 36

// ===== Encoder Pins =====
#define ENCODER_1 21
#define ENCODER_2 13

// ===== PWM Settings =====
#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES    LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY   5000
#define LEDC_CHANNEL_IN1 LEDC_CHANNEL_0
#define LEDC_CHANNEL_IN2 LEDC_CHANNEL_1

#define ENCODER_PPR 210

volatile int32_t encoder_count = 0;

// ===== UART Settings =====
#define UART_PORT_NUM  UART_NUM_0
#define UART_TX_PIN    UART_PIN_NO_CHANGE
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE  1024


portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;
static uhci_controller_handle_t uhci_ctrl = NULL;

extern volatile float SETPOINT_RPM_SHARED;
extern portMUX_TYPE setpoint_mux;

extern setpoint_mode_t current_mode;


// ===== CSV Queue =====
typedef struct {
    float rpm_unfiltered;
    float rpm_filtered;
    int pwm;
    int setpoint;
    uint32_t timestamp;
    float alpha;
} csv_msg_t;

#define CSV_QUEUE_LEN 64
static QueueHandle_t csv_queue = NULL;

// ===== UART init =====
void uart_init() {
    uart_config_t cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_PORT_NUM, &cfg);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uhci_controller_config_t uhci_cfg = {
        .uart_port = UART_PORT_NUM,
        .tx_trans_queue_depth = 30,
        .max_receive_internal_mem = 10 * 1024,
        .max_transmit_size = 10 * 1024,
        .dma_burst_size = 32,
        .rx_eof_flags.idle_eof = 1,
    };
    ESP_ERROR_CHECK(uhci_new_controller(&uhci_cfg, &uhci_ctrl));
}

static pcnt_unit_handle_t pcnt_unit = NULL;
static pcnt_channel_handle_t pcnt_chan = NULL;

void encoder_init(void)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = 10000,
        .low_limit  = -10000,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num  = ENCODER_2,  // count on this pin
        .level_gpio_num = ENCODER_1,  // direction from this pin
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // direction: if ENCODER_1 is HIGH, count forward; LOW → reverse
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        pcnt_chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,   // when level=1 invert sign
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE       // when level=0 normal
    ));

    // count every rising edge, ignore falling
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // rising
        PCNT_CHANNEL_EDGE_ACTION_HOLD        // falling
    ));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

// ===== Motor PWM Init =====
void motor_pwm_init(void) {
    ledc_timer_config_t t = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c1 = {
        .gpio_num = MOTOR_IN1_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_IN1,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };
    ledc_channel_config(&c1);

    ledc_channel_config_t c2 = {
        .gpio_num = MOTOR_IN2_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_IN2,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };
    ledc_channel_config(&c2);
}

// ===== CSV Sender Task =====
void csv_task(void *arg) {
    csv_msg_t msg;
    while (1) {
        if (xQueueReceive(csv_queue, &msg, portMAX_DELAY) == pdTRUE) {
            char buf[64];
            int len = snprintf(buf, sizeof(buf), "%d,%d,%d,%.3f\n",
                               (int)msg.rpm_filtered, msg.setpoint, msg.pwm, msg.alpha);
            uhci_transmit(uhci_ctrl, (uint8_t*)buf, len);
        }
    }
}




// ===== Motor Task (PI Control) =====
void motor_task(void *arg) {
    esp_task_wdt_add(NULL);
    const float Ts = 0.002f; // Control Loop: 2ms
    const float Ts_MEASURE = 0.010f; // Velocity Measurement: 10ms (5 * 2ms)
    const float rpm_factor_10ms = 60.0f / (ENCODER_PPR * Ts_MEASURE);

    // --- PID Control Parameters ---
    const float Kp = 16.0f; 
    const float Ki = 80.0f; 
    const float Kp_c = 0.5f; 
    const float Ki_c = 0.2f; 
    const float Kd = 0.0f;
    float setpoint = 50.0f;
    float prev_setpoint = 0.0f;

    float u_ff = 0.0f;
    float error_corr = 0.0f;
    
    // --- PID State Variables ---
    float integral_sum = 0.0f;
    float error_prev = 0.0f;
    const float MAX_DUTY = (1 << LEDC_DUTY_RES) - 1; // 8191 for 13-bit
    const float PWM_SAT_MAX = MAX_DUTY * 1.0f;
    const float PWM_SAT_MIN = PWM_SAT_MAX * -1.0f; 

    float last_pwm_val = 0.0f; 
    float current_duty_float = 0.0f;
    
    // --- Velocity Estimation State Variables ---
    int32_t last_count = 0; // The count from the previous 2ms tick
    int32_t delta_sum = 0;  // Sum of counts over 10ms
    float current_rpm = 0.0f; // The clean, 10ms-updated RPM value
    int tick_counter = 0;   // Counter for 5 ticks

    // --- Task Timing ---
    csv_msg_t msg;
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // 2ms in ticks
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // The rpm_factor for the raw 2ms data (used for logging only)
    const float rpm_factor_2ms = 60.0f / (ENCODER_PPR * Ts);

    while (1) {
        esp_task_wdt_reset();

        // 1. READ ENCODER COUNT
        int count_now = 0;
        pcnt_unit_get_count(pcnt_unit, &count_now);
        int32_t delta_2ms = count_now;
        pcnt_unit_clear_count(pcnt_unit);
        
        // Accumulate count for the 10ms window
        delta_sum += delta_2ms;

        // --- VELOCITY MEASUREMENT & FILTER (Runs every 10ms) ---
        if (++tick_counter >= 5) {
            tick_counter = 0;
            
            // Calculate the much cleaner, less-quantized RPM value
            float rpm_raw_10ms = (float)delta_sum * rpm_factor_10ms;

            static float prev_x = 0.0f;
            static float prev_y = 0.0f;

            float x = rpm_raw_10ms;                      // x[n]
            float y = 0.3375f * x
                    + 0.3375f * prev_x
                    + 0.3249f * prev_y;                  // y[n]

            prev_x = x;
            prev_y = y;

            current_rpm = y;
            
            delta_sum = 0; // Reset the 10ms count sum
 
        }
    
        taskENTER_CRITICAL(&setpoint_mux);
        setpoint = SETPOINT_RPM_SHARED;  // read current setpoint
        taskEXIT_CRITICAL(&setpoint_mux);
        float error = setpoint - current_rpm;

        float control_effort_clamped = 0.0f;
        #if CONTROL_MODE == MODE_PI_CONTROLLER

        float derivative_term = Kd * (error - error_prev) / Ts;
        
        
        float pd = Kp * error + derivative_term;
        error_prev = error;

        // 2. Compute integral limits and anti-windup (same as before)
        float I_max = PWM_SAT_MAX - pd;
        float I_min = PWM_SAT_MIN - pd;

        integral_sum += error * 0.01f;
        if (setpoint < 1.0f) {
            integral_sum = 0.0f;
            error_prev = 0.0f;
        }
        if (Ki * integral_sum > I_max) integral_sum = I_max / Ki;
        if (Ki * integral_sum < I_min) integral_sum = I_min / Ki;

        // 3. Final control effort and clamping
        float control_effort_raw = pd + Ki * integral_sum;

        control_effort_clamped = control_effort_raw;
        if (control_effort_clamped > PWM_SAT_MAX) {
            control_effort_clamped = PWM_SAT_MAX;
        } else if (control_effort_clamped < PWM_SAT_MIN) {
            control_effort_clamped = PWM_SAT_MIN;
        }
        
        #elif CONTROL_MODE == MODE_COMBINE

        u_ff = nn_predict_pwm(
            setpoint,        // omega_des
            current_rpm,     // omega_act
            last_pwm_val     // u_total_last_raw
        );

        error_corr = setpoint - current_rpm;

        // Standard PI integral calculation
        integral_sum += error_corr * 0.01f;

        float PI_LIMIT = 500.0f; 
        if (Ki_c * integral_sum > PI_LIMIT) integral_sum = PI_LIMIT / Ki_c;
        if (Ki_c * integral_sum < -PI_LIMIT) integral_sum = -PI_LIMIT / Ki_c;
      
        float u_pi = Kp_c * error_corr + Ki_c * integral_sum;


        /* ---------- COMBINED OUTPUT (u_total) ---------- */ 
        if (setpoint > 50.0f && fabs(error_corr) < 60.0f && fabs(error_corr) > 5.0f) { 
            alpha += GAMMA * error_corr * u_ff * 0.01f; 
        }
        else if (setpoint < 50.0f) {
            alpha = alpha * 0.999f + 1.0f * 0.001f;
        }
        float u = alpha * u_ff + u_pi;

        // absolute saturation
        if (u > PWM_SAT_MAX) u = PWM_SAT_MAX;
        if (u < PWM_SAT_MIN) u = PWM_SAT_MIN;

        control_effort_clamped = u;


        /* ---------- STATE UPDATE ---------- */
        error_prev = error; // This is the total error
        last_pwm_val = u; // Important: Record the final total PWM used for the next NN prediction

        if (fabs(setpoint - prev_setpoint) > 5.0f) {
            integral_sum = 0;
        }
        prev_setpoint = setpoint;

        #endif

        // Use the clamped control effort for PWM
        int pwm = (int)fabs(control_effort_clamped); 
        
        // 4. DRIVE MOTOR (Direction based on sign of control_effort_clamped)
        if (control_effort_clamped >= 0) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_IN1, pwm);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_IN2, 0);
        } else {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_IN1, 0);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_IN2, pwm);
        }

        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_IN1);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_IN2);
        
        // 5. PUSH CSV TO QUEUE
        // Log BOTH the noisy 2ms RPM and the clean 10ms RPM
        msg.rpm_unfiltered = (float)delta_2ms * rpm_factor_2ms; // Noisy 2ms RPM
        msg.rpm_filtered = current_rpm; // Clean 10ms RPM (for NN training)
        msg.pwm = pwm;
        msg.timestamp = (uint32_t)esp_timer_get_time() / 1000; // Log in milliseconds
        msg.setpoint = setpoint;
        msg.alpha = alpha;
        xQueueSend(csv_queue, &msg, 0); // non-blocking

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
// ===== Main =====
void app_main(void) {
    uart_init();
    motor_pwm_init();
    encoder_init();

    csv_queue = xQueueCreate(CSV_QUEUE_LEN, sizeof(csv_msg_t));
    xTaskCreate(csv_task, "csv_sender", 2048, NULL, 5, NULL);
    xTaskCreate(setpoint_task, "setpoint", 2048, NULL, 7, NULL);
    xTaskCreatePinnedToCore(motor_task, "motor", 4096, NULL, 10, NULL, 1);
}

