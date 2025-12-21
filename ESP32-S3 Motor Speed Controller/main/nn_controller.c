#include "nn_controller.h"
#include "motor_nn_weights.h" 
#include <math.h> // For basic math operations
#include <string.h>
float PWM_MIN = 0.0f;
float PWM_MAX = 8192.0f;
float RPM_MIN = 0.0f;
float RPM_MAX = 635.0f;

float PWM_RANGE = 8192.0f; // 8192.0f
float RPM_RANGE = 635.0f; // 635.0f

int INPUT_DIM = 3;
int HIDDEN_UNITS = 20;
int OUTPUT_DIM = 1;


// Helper function for ReLU activation
static inline float relu(float x) {
    // The ReLU function: max(0, x)
    return (x > 0.0f) ? x : 0.0f;
}

// Helper function for normalization: (value - MIN) / RANGE
static inline float normalize(float value, float min_val, float range_val) {
    return (value - min_val) / range_val;
}

// Helper function for de-normalization: (value_norm * RANGE) + MIN
static inline float unscale(float value_norm, float min_val, float range_val) {
    return (value_norm * range_val) + min_val;
}


float nn_predict_pwm(float omega_des, float omega_act, float u_total_last_raw) {
    
    // --- 1. Normalize the Inputs ---
    float input_norm[INPUT_DIM];
    input_norm[0] = normalize(omega_des, RPM_MIN, RPM_RANGE);
    input_norm[1] = normalize(omega_act, RPM_MIN, RPM_RANGE);
    input_norm[2] = normalize(u_total_last_raw, PWM_MIN, PWM_RANGE);

    // --- 2. Layer 1 (3 -> 20) with ReLU ---
    float h1[HIDDEN_UNITS];
    memset(h1, 0, sizeof(h1)); 
    for (int j = 0; j < HIDDEN_UNITS; j++) {
        float sum = 0.0f;
        for (int i = 0; i < INPUT_DIM; i++) {
            sum += input_norm[i] * w1_weights[i][j]; // Wx
        }
        sum += b1_biases[j]; // Wx + b
        h1[j] = relu(sum);
    }

    // --- 3. Layer 2 (20 -> 20) with ReLU ---
    float h2[HIDDEN_UNITS];
    memset(h2, 0, sizeof(h2)); 
    for (int j = 0; j < HIDDEN_UNITS; j++) {
        float sum = 0.0f;
        for (int i = 0; i < HIDDEN_UNITS; i++) {
            sum += h1[i] * w2_weights[i][j];
        }
        sum += b2_biases[j];
        h2[j] = relu(sum);
    }

    // --- 4. Output Layer (20 -> 1) with Linear Activation ---
    float output_norm = 0.0f; 
    for (int i = 0; i < HIDDEN_UNITS; i++) {
        output_norm += h2[i] * w3_weights[i][0];
    }
    output_norm += b3_biases[0];
    
    // --- 5. De-Normalize and Clamp the Output ---
    float predicted_pwm = unscale(output_norm, PWM_MIN, PWM_RANGE);

    // Clamp the output to ensure it stays within physical limits (0-8192)
    if (predicted_pwm > PWM_MAX) predicted_pwm = PWM_MAX;
    if (predicted_pwm < PWM_MIN) predicted_pwm = PWM_MIN;

    return predicted_pwm;
}