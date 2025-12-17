#ifndef NN_CONTROLLER_H
#define NN_CONTROLLER_H

// --- 1. Fixed Normalization Limits (MUST MATCH PYTHON CELL 1) ---
extern float PWM_MIN;
extern float PWM_MAX;
extern float RPM_MIN;
extern float RPM_MAX;
extern float PWM_RANGE;
extern float RPM_RANGE;


// // --- 2. NN Model Constants ---
// const int INPUT_DIM = 3;
// const int HIDDEN_UNITS = 16;
// const int OUTPUT_DIM = 1;


// ===== ANN Constants =====
extern int INPUT_DIM;
extern int HIDDEN_UNITS;
extern int OUTPUT_DIM;

/**
 * @brief Predicts the required feedforward PWM for the given motor state.
 * @return float Predicted feedforward PWM (0-8192).
 */
// float nn_predict_pwm(float omega_des, float omega_act, float u_total_last_raw);
float nn_predict_pwm(float omega_des, float omega_act, float u_total_last_raw);
#endif // NN_CONTROLLER_H