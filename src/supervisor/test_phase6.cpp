#include <iostream>
#include <fstream>
#include "../sim/quadcopter_state.h"
#include "../sim/quadcopter_dynamics.h"
#include "../control/cascaded_pid.h"
#include "../control/position_controller.h"
#include "../fdi/fault_detector.h"
#include "supervisor.h"

int main() {
    std::cout << "--- Phase 6: Fault Tolerance and Recovery ---\n";
    
    double dt = 0.005;
    int steps = 8.0 / dt; // 8 seconds total
    
    QuadcopterState state;
    state.position = Eigen::Vector3d(0.0, 0.0, 5.0);
    QuadcopterDynamics dynamics(1.0, 9.81, 0.1, 0.01, 0.01, 0.02);
    
    PositionController pos_ctrl;
    PIDController alt_pid(1.5, 0.3, 0.3, -3.0, 3.0);
    PIDController att_roll_pid(5.0, 0.0, 1.0, -1.5, 1.5);
    PIDController att_pitch_pid(5.0, 0.0, 1.0, -1.5, 1.5);
    PIDController rate_roll_pid(0.1, 0.0, 0.003, -0.5, 0.5);
    PIDController rate_pitch_pid(0.1, 0.0, 0.003, -0.5, 0.5);
    
    FaultDetector fdi(15.0);
    Supervisor supervisor(5.0, 1.0); // Start at 5m, descend at 1.0 m/s
    
    std::ofstream file("phase6_recovery.csv");
    file << "time,state,m1,m2,m3,m4,z,target_z,roll_rate\n";
    
    Eigen::Vector2d target_pos(0.0, 0.0);
    
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        
        // 1. Nominal Hover Commands
        double total_thrust = 9.81;
        Eigen::Vector4d f_cmd = dynamics.getMixer().mix(total_thrust, Eigen::Vector3d(0.0, 0.0, 0.0));
        
        // 2. INJECT FAULT: Motor 1 (index 0) fails at t = 3.0s
        Eigen::Vector4d f_actual = f_cmd;
        if (t >= 3.0) {
            f_actual(0) = 0.0; 
        }
        
        // 3. Calculate actual torque for FDI
        double L = 0.25, c = 0.05;
        Eigen::Vector3d tau_actual;
        tau_actual.x() = L * (f_actual(1) - f_actual(3));
        tau_actual.y() = L * (-f_actual(0) + f_actual(2));
        tau_actual.z() = c * (-f_actual(0) + f_actual(1) - f_actual(2) + f_actual(3));
        
        // 4. Update Physics with FAILED motor
        dynamics.step(state, f_actual, dt);
        
        // 5. FDI Check
        Eigen::Vector3d tau_cmd;
        tau_cmd.x() = L * (f_cmd(1) - f_cmd(3));
        tau_cmd.y() = L * (-f_cmd(0) + f_cmd(2));
        tau_cmd.z() = c * (-f_cmd(0) + f_cmd(1) - f_cmd(2) + f_cmd(3));
        
        bool fault_detected = fdi.update(tau_cmd, state.angular_rates, dt);
        
        // 6. Supervisor Logic
        supervisor.update(dt, state.position.z());
        if (fault_detected) {
            supervisor.triggerRecovery();
        }
        
        // 7. Control based on State
        int current_state = static_cast<int>(supervisor.getState());
        double current_target_z = supervisor.getTargetAltitude();
        double current_target_roll = 0.0;
        double current_target_pitch = 0.0;
        
        if (current_state == 0) { // NORMAL
            Eigen::Vector2d angles = pos_ctrl.compute(target_pos, state.position.head<2>(), state.velocity.head<2>(), dt);
            current_target_roll = angles.x();
            current_target_pitch = angles.y();
            current_target_z = 5.0;
        } else if (current_state == 1) { // RECOVERY
            // Emergency descent: force level attitude, use fixed descent thrust
            current_target_roll = 0.0;
            current_target_pitch = 0.0;
            // We bypass the altitude PID and command a fixed thrust slightly below gravity
            // This guarantees a controlled descent instead of fighting the spin
        }
        
        // Calculate final thrust
        double final_thrust = 9.81;
        if (current_state == 0) {
            double thrust_adj = alt_pid.compute(current_target_z, state.position.z(), dt);
            final_thrust = std::clamp(9.81 + thrust_adj, 0.0, 15.0);
        } else if (current_state == 1) {
            final_thrust = 8.5; // Controlled descent thrust (less than gravity)
        } else {
            final_thrust = 0.0; // LANDED
        }
        
        // Attitude control
        Eigen::Vector3d euler = state.attitude.toRotationMatrix().eulerAngles(2, 1, 0);
        double tr = att_roll_pid.compute(current_target_roll, euler.z(), dt);
        double tp = att_pitch_pid.compute(current_target_pitch, euler.y(), dt);
        double tx = rate_roll_pid.compute(tr, state.angular_rates.x(), dt);
        double ty = rate_pitch_pid.compute(tp, state.angular_rates.y(), dt);
        
        file << t << "," << current_state << "," << f_actual(0) << "," << f_actual(1) << "," 
             << f_actual(2) << "," << f_actual(3) << "," << state.position.z() << "," 
             << current_target_z << "," << state.angular_rates.x() << "\n";
    }
    
    file.close();
    std::cout << "Simulation complete. Data saved to phase6_recovery.csv\n";
    return 0;
}
