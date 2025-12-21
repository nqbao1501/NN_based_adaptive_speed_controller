#include <systemc>
#include <iostream>
#include <cmath>    
#include <cstdlib>  
#include <ctime> 
#include <fstream>
#include "initial_NN_6_inputs.h"
using namespace sc_core;
using namespace std;

// === CONSTANTS MATCHING motor_speed_controller.c ===
// Ts = 0.002f (2ms) from motor_task
#define SYSTEM_TIME_UNIT sc_time(2, SC_MS) 

// PID Gains from motor_task
#define KP 17.0 
#define KI 72.5
#define KD 0.0 // Derivative term is 0.0f in motor_task


// ANN Structure (As defined previously)
#define INPUT_SIZE 6    // [Error, Current_RPM, Target_RPM]
#define HIDDEN_SIZE_1 16
#define HIDDEN_SIZE_2 16
#define HIDDEN_SIZE_3 16
#define OUTPUT_SIZE 1    


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

        /*for(int i=1; i<3; ++i) y_states[i] = 0.0;
        for(int i=0; i<5; ++i) u_history[i] = 0.0;*/

        y_states[0] = 451.132;
        y_states[1] = 451.133;
        y_states[2] = 451.134;
        u_history[0] = 221.125;
        u_history[1] = 221.276;
        u_history[2] = 221.427;
        u_history[3] = 221.579;
        u_history[4] = 221.73;

        log_file.open("/home/vietanh/Visual_Studio_for_Python/rpm_pwm_log_NN_0_RPM.csv");
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

// === CONTROL MODE ENUM ===
enum ControlMode {
    PID_MODE,
    ANN_MODE
};

// === ANN CONTROLLER STRUCTURE ===
struct ANNController {

    inline double relu(double x) {
        return x > 0 ? x : 0;
    }

    inline double sigmoid(double x) {
    return 1.0 / (1.0 + exp(-x));
}

    double calculate(double error, double current_rpm, double target_rpm, double integral, double last_error, double last_pwm) {

        double input[INPUT_SIZE] = {error, current_rpm, target_rpm, integral, last_error, last_pwm};

        double h1[HIDDEN_SIZE_1];
        double h2[HIDDEN_SIZE_2];
        double h3[HIDDEN_SIZE_3];
        double out[OUTPUT_SIZE];

        // Layer 1
        for (int j = 0; j < HIDDEN_SIZE_1; ++j) {
            double sum = B1[j];
            for (int i = 0; i < INPUT_SIZE; ++i)
                sum += input[i] * W1[i][j];
            h1[j] = relu(sum);
        }

        // Layer 2
        for (int k = 0; k < HIDDEN_SIZE_2; ++k) {
            double sum = B2[k];
            for (int j = 0; j < HIDDEN_SIZE_1; ++j)
                sum += h1[j] * W2[j][k];
            h2[k] = relu(sum);
        }

        // Output
        for (int o = 0; o < HIDDEN_SIZE_3; ++o) {
            double sum = B3[o];
            for (int k = 0; k < HIDDEN_SIZE_2; ++k)
                sum += h2[k] * W3[k][o];
            h3[o] = relu(sum);
        }

        for (int p = 0; p < OUTPUT_SIZE; ++p) {
            double sum = B4[p];
            for (int k = 0; k < HIDDEN_SIZE_3; ++k)
                sum += h3[k] * W4[k][p];
            out[p] = sum;
        }
        return out[0];
    }
};



// === SYSTEM CONTROLLER (PID/ANN LOGIC) ===
SC_MODULE(SystemController) {
    sc_in<double> rpm_in;
    sc_out<double> duty_cycle_out;

    // Use defines for constants
    double Kp = KP;
    double Ki = KI;
    double Kd = KD;

    double setpoint_rpm;
    double integral_sum;
    double last_error;
    double last_pwm;
    double current_duty = 0.0;
    double delta_duty = 0.0;

    ControlMode current_mode;
    ANNController ann_instance;
    
    // Max PWM duty corresponds to 1.0 (100%) duty cycle in simulation
    const double PWM_SAT_MAX = 8191.0; 
    const double PWM_SAT_MIN = -8191.0; 


    SC_CTOR(SystemController) {
        SC_THREAD(controller_thread);
        setpoint_rpm = 50.0; 
        integral_sum = 0.0;
        last_error = 0.0;
        last_pwm = 0.0;
        current_mode = ANN_MODE; // Start with ANN
    }

    void controller_thread() {
        // Ts is the control period
        double dt = SYSTEM_TIME_UNIT.to_seconds(); 

        duty_cycle_out.write(0.0);
        wait(SYSTEM_TIME_UNIT); 

        wait(sc_time(100, SC_MS)); 
        
        cout << "\nController Initialized (Ts: " << dt * 1000 << "ms)" << endl;

        while (true) {
            // Wait for 2ms control cycle (matching the ESP32 task)
            wait(SYSTEM_TIME_UNIT);

            double current_rpm = rpm_in.read();
            double error = setpoint_rpm - current_rpm;

            if (current_mode == PID_MODE) {
                // --- PID Logic (PI Control, as Kd is 0.0f) ---

                double proportional_term = Kp * error;
                integral_sum += error * dt;
                
                // Kd is 0.0, so derivative_term is 0.0
                double derivative_term = Kd * (error - last_error) / dt; 

                double control_effort_raw = proportional_term + (Ki * integral_sum) + derivative_term;
                
                // Anti-Windup Logic (Matching the logic in motor_task: revert integral addition)
                if (control_effort_raw > PWM_SAT_MAX) {
                    integral_sum -= error * dt; 
                    control_effort_raw = PWM_SAT_MAX;
                } else if (control_effort_raw < PWM_SAT_MIN) {
                    integral_sum -= error * dt; 
                    control_effort_raw = PWM_SAT_MIN;
                }
                
                // Update State for the next iteration
                last_error = error;

                current_duty = control_effort_raw; 
                
                // Clamp final duty output, although it should be clamped by anti-windup
                if (current_duty > PWM_SAT_MAX) current_duty = PWM_SAT_MAX;
                if (current_duty < PWM_SAT_MIN) current_duty = PWM_SAT_MIN;

            }
            else if (current_mode == ANN_MODE) {
                // ANN Logic (Using placeholder structure)
                double last_error_n   = last_error / 635.0;
                double error_n        = error / 635.0;
                double current_rpm_n  = current_rpm / 635.0;
                double setpoint_rpm_n = setpoint_rpm / 635.0;
                double last_pwm_n     = last_pwm / 8191.0;

                double integral_n = integral_sum / 635.0;  

                /* --- ANN inference --- */
                double ann_output = ann_instance.calculate(
                    error_n,
                    current_rpm_n,
                    setpoint_rpm_n,
                    integral_n,
                    last_error_n,
                    last_pwm_n
                );

                /* --- Scale ANN output --- */
                integral_sum += 0.01 * error;
                delta_duty = ann_output * 8191.0;
                current_duty += delta_duty;

                // Simple clamping for ANN output
                if (current_duty > PWM_SAT_MAX) current_duty = PWM_SAT_MAX;
                if (current_duty < PWM_SAT_MIN) current_duty = PWM_SAT_MIN;

                last_error = error;
                last_pwm = current_duty;
            }

            duty_cycle_out.write(current_duty);
            
            const char* mode_name = (current_mode == PID_MODE) ? "PID" : "ANN";

            cout << sc_time_stamp() << " -> CTRL [" << mode_name << "]: Error=" << error << ", Duty=" << current_duty << ", RPM=" << current_rpm << endl;
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

    double setpoints[] = {0};
    int num_steps = sizeof(setpoints) / sizeof(setpoints[0]);

    for (int i = 0; i < num_steps; ++i) {
        // Update the setpoint in the controller
        ctrl_inst.setpoint_rpm = setpoints[i];
        
        // Run for a specific duration (e.g., 500ms per step)
        sc_start(sc_time(10000, SC_MS)); 
    }


    cout << "Simulation finished." << endl;

    return 0;
}

