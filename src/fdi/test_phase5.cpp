#include <iostream>
#include <fstream>
#include "../sim/quadcopter_state.h"
#include "../sim/quadcopter_dynamics.h"
#include "../control/cascaded_pid.h"
#include "../fdi/fault_detector.h"

int main() {
    std::cout << "--- Phase 5: Fault Detection & Isolation ---\n";
    
    double dt = 0.005;
    int steps = 6.0 / dt; // 6 seconds total
    
    QuadcopterState state;
    state.position = Eigen::Vector3d(0.0, 0.0, 5.0);
    QuadcopterDynamics dynamics(1.0, 9.81, 0.1, 0.01, 0.01, 0.02);
    
    // Simple hover controller
    PIDController alt_pid(1.5, 0.3, 0.3, -3.0, 3.0);
    PIDController att_roll_pid(5.0, 0.0, 1.0, -1.5, 1.5);
    PIDController att_pitch_pid(5.0, 0.0, 1.0, -1.5, 1.5);
    PIDController rate_roll_pid(0.1, 0.0, 0.003, -0.5, 0.5);
    PIDController rate_pitch_pid(0.1, 0.0, 0.003, -0.5, 0.5);
    
    FaultDetector fdi(15.0); // Threshold tuned for this 1kg drone
    
    std::ofstream file("phase5_fdi.csv");
    file << "time,m1,m2,m3,m4,roll_rate,residual,fault_state\n";
    
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        
        // 1. Nominal Hover Commands
        double total_thrust = 9.81;
        double target_torque_x = 0.0;
        double target_torque_y = 0.0;
        
        // Get commanded motor forces from mixer
        Eigen::Vector4d f_cmd = dynamics.getMixer().mix(total_thrust, Eigen::Vector3d(target_torque_x, target_torque_y, 0.0));
        
        // 2. INJECT FAULT: Motor 1 (index 0) fails at t = 3.0s
        Eigen::Vector4d f_actual = f_cmd;
        if (t >= 3.0) {
            f_actual(0) = 0.0; // Motor 1 dies completely
        }
        
        // 3. Calculate actual torque produced by the failing motors (for FDI residual calculation)
        double L = 0.25, c = 0.05;
        Eigen::Vector3d tau_actual;
        tau_actual.x() = L * (f_actual(1) - f_actual(3));
        tau_actual.y() = L * (-f_actual(0) + f_actual(2));
        tau_actual.z() = c * (-f_actual(0) + f_actual(1) - f_actual(2) + f_actual(3));
        
        // 4. Update Physics with FAILED motors
        dynamics.step(state, f_actual, dt);
        
        // 5. Run Fault Detector
        // We compare the COMMANDED torque (what the controller thinks is happening) 
        // vs the MEASUREED angular rates (what is actually happening)
        Eigen::Vector3d tau_cmd;
        tau_cmd.x() = L * (f_cmd(1) - f_cmd(3));
        tau_cmd.y() = L * (-f_cmd(0) + f_cmd(2));
        tau_cmd.z() = c * (-f_cmd(0) + f_cmd(1) - f_cmd(2) + f_cmd(3));
        
        bool fault_detected = fdi.update(tau_cmd, state.angular_rates, dt);
        double residual = fdi.isFault() ? 15.0 : (tau_cmd - tau_actual).norm(); // Approximate residual for plotting
        
        int fault_state = fault_detected ? 1 : 0;
        
        // Log to CSV
        file << t << "," << f_actual(0) << "," << f_actual(1) << "," << f_actual(2) << "," << f_actual(3) << ","
             << state.angular_rates.x() << "," << residual << "," << fault_state << "\n";
    }
    
    file.close();
    std::cout << "Simulation complete. Data saved to phase5_fdi.csv\n";
    return 0;
}
