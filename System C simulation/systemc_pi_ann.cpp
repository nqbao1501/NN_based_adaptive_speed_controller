#include <systemc>
#include <iostream>
#include <cmath>    
#include <cstdlib>  
#include <ctime> 
#include <fstream>
#include "motor_nn_weights.h"
#include <algorithm>
using namespace sc_core;
using namespace std;

// === CONSTANTS MATCHING motor_speed_controller.c ===
// Ts = 0.002f (2ms) from motor_task
#define SYSTEM_TIME_UNIT sc_time(2, SC_MS) 

// PID Gains from motor_task
#define KP 17.0 
#define KI 72.5
#define KD 0.0 // Derivative term is 0.0f in motor_task
#define KP_C 0.12f
#define KI_C 0.1f
#define GAMMA 0.0000001f
#define PI_LIMIT 500.0f

#define INPUT_DIM 3
#define HIDDEN_UNITS 20
#define OUTPUT_DIM 1

// Normalization Constants
const float PWM_MIN = 0.0f;
const float PWM_MAX = 8192.0f;
const float RPM_MIN = 0.0f;
const float RPM_MAX = 635.0f;
const float PWM_RANGE = 8192.0f;
const float RPM_RANGE = 635.0f; 


// === BLDC ENGINE MODEL (The Plant) ===
SC_MODULE(BldcEngine) {
    sc_in<double> duty_cycle_in;
    sc_out<double> rpm_out;
    std::ofstream log_file;
    // *** Z-DOMAIN COEFFICIENTS (Denominator) ***
    const double Z_A1 = 2.8025; 
    const double Z_A2 = -2.6071;
    const double Z_A3 = 0.8046;

    // *** Z-DOMAIN COEFFICIENTS (Numerator) ***
    const double Z_B0 = 0.0003133;
    const double Z_B1 = -0.0004699;
    const double Z_B2 = 0.0001567;

    // Output State Variables: y[n-1], y[n-2], y[n-3]
    double y_states[3]; 

    // Input State Variables: Buffer for u[n-1] through u[n-5] 
    // We need 5 steps of history for the delayed system
    double u_history[5]; 
    
    // Indices for easy reading:
    // u[n-2] is u_history[1]
    // u[n-3] is u_history[2]
    // u[n-4] is u_history[3]
    // u[n-5] is u_history[4]
    
    SC_CTOR(BldcEngine) {
        // Initialize all states to zero

        for(int i=1; i<3; ++i) y_states[i] = 0.0;
        for(int i=0; i<5; ++i) u_history[i] = 0.0;

        //y_states[0] = 451.132;
        //y_states[1] = 451.133;
        //y_states[2] = 451.134;
        //u_history[0] = 221.125;
        //u_history[1] = 221.276;
        //u_history[2] = 221.427;
        //u_history[3] = 221.579;
        //u_history[4] = 221.73;
        log_file.open("/home/vietanh/Visual_Studio_for_Python/rpm_pwm_log.csv");
        log_file << "time_sec,rpm,pwm\n";
        SC_THREAD(plant_update_thread);
    }

    void plant_update_thread() {
        while (true) {
            wait(SYSTEM_TIME_UNIT);

            // 1. Read the current input u[n]
            double u_curr = duty_cycle_in.read();
            // Clamp input
            if (u_curr < -8191.0) u_curr = -8191.0;
            if (u_curr > 8191.0) u_curr = 8191.0;

            // 2. The Total Difference Equation:
            // The terms b0*u[n] + b1*u[n-1] + ... are now applied to the delayed inputs:
            
            // The effective input to the B1 term is u[n-3] (from the z^-2 delay + z^-1)
            double u_n_minus_3 = u_history[2];
            // The effective input to the B2 term is u[n-4] (from the z^-2 delay + z^-2)
            double u_n_minus_4 = u_history[3];
            // The effective input to the B3 term is u[n-5] (from the z^-2 delay + z^-3)
            double u_n_minus_5 = u_history[4];

            double y_curr = 
                // Recursive Output Terms (Poles)
                  (Z_A1 * y_states[0]) // y[n-1]
                + (Z_A2 * y_states[1]) // y[n-2] 
                + (Z_A3 * y_states[2]) // y[n-3]
                // Delayed Input Terms (Zeros)
                + (Z_B0 * u_n_minus_3) 
                + (Z_B1 * u_n_minus_4) 
                + (Z_B2 * u_n_minus_5);
            
            
            // Shift output states: y[n-3] <- y[n-2], y[n-2] <- y[n-1], y[n-1] <- y[n]
            y_states[2] = y_states[1];
            y_states[1] = y_states[0];
            y_states[0] = y_curr;

            // Shift input states: u[n-5] <- u[n-4] ... u[n-1] <- u[n]
            for(int i=4; i>0; --i) u_history[i] = u_history[i-1];
            u_history[0] = u_curr;
            
            // 4. Output the current RPM (y[n])
            rpm_out.write(y_curr);
            
            cout << sc_time_stamp() << " -> Engine: Duty=" << u_curr << ", RPM=" << y_curr << endl;
            log_file << sc_time_stamp().to_seconds() << "," << y_curr << "," << u_curr << "\n";

        }
    }
};

// === ANN CONTROLLER STRUCTURE ===
struct ANNController {
    inline float relu(float x) { return (x > 0.0f) ? x : 0.0f; }
    inline float normalize(float value, float min_val, float range_val) { return (value - min_val) / range_val; }
    inline float unscale(float value_norm, float min_val, float range_val) { return (value_norm * range_val) + min_val; }

    // Matches nn_predict_pwm mechanism
    double calculate(float omega_des, float omega_act, float u_total_last_raw) {
        // 1. Normalization
        float input_norm[INPUT_DIM];
        input_norm[0] = normalize(omega_des, RPM_MIN, RPM_RANGE);
        input_norm[1] = normalize(omega_act, RPM_MIN, RPM_RANGE);
        input_norm[2] = normalize(u_total_last_raw, PWM_MIN, PWM_RANGE);

        // 2. Layer 1 (3 -> 20) with ReLU
        float h1[HIDDEN_UNITS];
        std::fill(std::begin(h1), std::end(h1), 0.0);
        for (int j = 0; j < HIDDEN_UNITS; j++) {
            float sum = 0.0f;
            for (int i = 0; i < INPUT_DIM; i++) {
                sum += input_norm[i] * w1_weights[i][j]; 
            }
            sum += b1_biases[j];
            h1[j] = relu(sum);
        }

        // 3. Layer 2 (20 -> 20) with ReLU
        float h2[HIDDEN_UNITS];
        std::fill(std::begin(h2), std::end(h2), 0.0);
        for (int j = 0; j < HIDDEN_UNITS; j++) {
            float sum = 0.0f;
            for (int i = 0; i < HIDDEN_UNITS; i++) {
                sum += h1[i] * w2_weights[i][j];
            }
            sum += b2_biases[j];
            h2[j] = relu(sum);
        }

        // 4. Output Layer (20 -> 1) Linear
        float output_norm = 0.0f; 
        for (int i = 0; i < HIDDEN_UNITS; i++) {
            output_norm += h2[i] * w3_weights[i][0];
        }
        output_norm += b3_biases[0];
        
        // 5. De-Normalization and Clamping
        float predicted_pwm = unscale(output_norm, PWM_MIN, PWM_RANGE);
        if (predicted_pwm > PWM_MAX) predicted_pwm = PWM_MAX;
        if (predicted_pwm < PWM_MIN) predicted_pwm = PWM_MIN;

        return (double)predicted_pwm;
    }
};

// === SYSTEM CONTROLLER (PID/COMBINE LOGIC) ===
enum ControlMode { PID_MODE, COMBINE_MODE };

SC_MODULE(SystemController) {
    sc_in<double> rpm_in;
    sc_out<double> duty_cycle_out;

    double setpoint_rpm;
    double prev_setpoint_rpm;
    double integral_sum;
    double last_error;
    double last_pwm;
    double alpha; 
    ControlMode current_mode;
    ANNController ann_instance;

    SC_CTOR(SystemController) {
        SC_THREAD(controller_thread);
        setpoint_rpm = 0.0; 
        prev_setpoint_rpm = 0.0;
        integral_sum = 0.0;
        last_error = 0.0;
        last_pwm = 0.0;
        alpha = 1.0; 
        current_mode = COMBINE_MODE; // Toggle this as needed
    }

    void controller_thread() {
        double dt = SYSTEM_TIME_UNIT.to_seconds(); 
        duty_cycle_out.write(0.0);
        wait(sc_time(100, SC_MS)); 

        while (true) {
            wait(SYSTEM_TIME_UNIT);

            double current_rpm = rpm_in.read();
            double error = setpoint_rpm - current_rpm;
            double current_duty = 0.0;

            if (current_mode == PID_MODE) {
                // --- ORIGINAL PID LOGIC ---
                double proportional_term = KP * error;
                integral_sum += error * dt;
                double control_effort_raw = proportional_term + (KI * integral_sum);
                
                // Anti-Windup
                if (control_effort_raw > 8191.0) {
                    integral_sum -= error * dt; 
                    control_effort_raw = 8191.0;
                } else if (control_effort_raw < -8191.0) {
                    integral_sum -= error * dt; 
                    control_effort_raw = -8191.0;
                }
                current_duty = control_effort_raw;

            } else if (current_mode == COMBINE_MODE) {
                // --- COMBINE MODE LOGIC ---
                if (std::abs(setpoint_rpm - prev_setpoint_rpm) > 5) {
                    integral_sum = 0;
                }
                prev_setpoint_rpm = setpoint_rpm;
                // 1. ANN Feedforward (u_ff)
                double u_ff = ann_instance.calculate((float)setpoint_rpm, (float)current_rpm, (float)last_pwm);

                // 2. PI Corrector (u_pi)
                integral_sum += error * 0.01; // Scale matching firmware
                if (KI_C * integral_sum > PI_LIMIT) integral_sum = PI_LIMIT / KI_C;
                if (KI_C * integral_sum < -PI_LIMIT) integral_sum = -PI_LIMIT / KI_C;
                double u_pi = (KP_C * error) + (KI_C * integral_sum);

                // 3. Adaptive Gain Update
                if (setpoint_rpm > 50.0 && std::abs(error) < 60.0 && std::abs(error) > 5.0) {
                    alpha += GAMMA * error * u_ff * 0.01;
                } else if (setpoint_rpm < 50.0) {
                    alpha = alpha * 0.999 + 1.0 * 0.001;
                }
                current_duty = (alpha * u_ff) + u_pi; 
                if (alpha > 1.33f) alpha = 1.33f; 
                if (alpha < 0.67f) alpha = 0.67f;
                cout << sc_time_stamp() << " -> Controller: Duty=" << current_duty << " NN Duty: " << u_ff << " x alpha: " << alpha << " + PI duty: " << u_pi <<" Error: " << error << " Integral: " << integral_sum << endl;

            }
            // Safety Clamp and State Update
            if (current_duty > 8191.0) current_duty = 8191.0;
            if (current_duty < -8191.0) current_duty = -8191.0;
            
            last_pwm = current_duty;
            last_error = error;
            duty_cycle_out.write(current_duty);
            
        }
    }
};
// === TESTBENCH ===
int sc_main(int argc, char* argv[]) {
    srand(time(NULL)); 
    sc_signal<double> duty_cycle_signal;
    sc_signal<double> rpm_signal;   // New signal for filtered data

    BldcEngine engine_inst("engine");
    SystemController ctrl_inst("controller");

    // 1. Controller Output -> Engine Input
    ctrl_inst.duty_cycle_out(duty_cycle_signal);
    engine_inst.duty_cycle_in(duty_cycle_signal);

    // 2. Engine Output (UNFILTERED) -> Filter Input
    engine_inst.rpm_out(rpm_signal); 
    ctrl_inst.rpm_in(rpm_signal); // The controller now reads filtered data

    cout << "Starting Simulation with Dynamic Setpoints..." << endl;

    // Array of setpoints from your graph (approximate values)
    double setpoints[] = {450};
    int num_steps = sizeof(setpoints) / sizeof(setpoints[0]);

    for (int i = 0; i < num_steps; ++i) {
        // Update the setpoint in the controller
        ctrl_inst.setpoint_rpm = setpoints[i];
        
        // Run for a specific duration (e.g., 500ms per step)
        sc_start(sc_time(25000, SC_MS)); 
    }

    cout << "Simulation finished." << endl;
    return 0;
}


