#include "external_setpoint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

/* ================= CONFIG ================= */

#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_1     // GPIO2 on ESP32-S3
#define ADC_ATTEN       ADC_ATTEN_DB_11
#define ADC_BITWIDTH    ADC_BITWIDTH_12

#define ADC_SAMPLES     16
#define UPDATE_PERIOD   20                // ms

#define RPM_MIN         50.0f
#define RPM_MAX         650.0f

#define FILTER_ALPHA    0.05f
#define RPM_SLEW_RPS    500.0f

/* ========================================== */

static const char *TAG = "external_setpoint";

volatile float external_setpoint_rpm = 0.0f;

static float filtered = 0.0f;
static float last_rpm = 0.0f;

/* ADC handles */
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle = NULL;
static bool adc_calibrated = false;

/* -------- TASK -------- */

static void external_setpoint_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    while (1) {

        int raw;
        uint32_t sum = 0;

        /* Oversample ADC */
        for (int i = 0; i < ADC_SAMPLES; i++) {
            adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);
            sum += raw;
        }

        int raw_avg = sum / ADC_SAMPLES;
        float adc_mv;

        /* Apply calibration if available */
        if (adc_calibrated) {
            int mv;
            adc_cali_raw_to_voltage(cali_handle, raw_avg, &mv);
            adc_mv = (float)mv;
        } else {
            /* Fallback conversion */
            adc_mv = raw_avg * (3300.0f / 4095.0f);
        }

        /* Map voltage → RPM */
        float rpm = RPM_MIN +
            (adc_mv / 3300.0f) * (RPM_MAX - RPM_MIN);

        /* Clamp */
        if (rpm < RPM_MIN) rpm = RPM_MIN;
        if (rpm > RPM_MAX) rpm = RPM_MAX;

        /* IIR filter */
        filtered += FILTER_ALPHA * (rpm - filtered);

        /* Slew-rate limit */
        float dt = UPDATE_PERIOD / 1000.0f;
        float max_step = RPM_SLEW_RPS * dt;

        if (filtered > last_rpm + max_step)
            filtered = last_rpm + max_step;
        else if (filtered < last_rpm - max_step)
            filtered = last_rpm - max_step;

        last_rpm = filtered;
        external_setpoint_rpm = filtered;

        vTaskDelayUntil(&last, pdMS_TO_TICKS(UPDATE_PERIOD));
    }
}

/* -------- INIT -------- */

void external_setpoint_init(void)
{
    /* -------- ADC ONESHOT INIT -------- */

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg)
    );

    /* -------- ADC CALIBRATION -------- */

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT,
        .chan = ADC_CHANNEL,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };

    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
        adc_calibrated = true;
        ESP_LOGI(TAG, "ADC calibration enabled (curve fitting)");
    } else {
        ESP_LOGW(TAG, "ADC calibration not available, using fallback");
    }

    /* -------- TASK CREATE -------- */

    xTaskCreatePinnedToCore(
        external_setpoint_task,
        "external_setpoint",
        2048,
        NULL,
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "External setpoint (ADC oneshot) initialized");
}
