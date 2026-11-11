#include <iostream>
#include <fstream>
#include "../sim/quadcopter_state.h"
#include "../sim/quadcopter_dynamics.h"
#include "cascaded_pid.h"
#include "position_controller.h"

int main() {
    std::cout << "--- Phase 3: Waypoint Tracking & Wind Rejection ---\n";
    double dt = 0.005;
    int steps = 10.0 / dt;
    
    QuadcopterState state;
    state.position = Eigen::Vector3d(0.0, 0.0, 5.0);
    QuadcopterDynamics dynamics(1.0, 9.81, 0.1, 0.01, 0.01, 0.02);
    
    // Use default physics-based gains
    PositionController pos_ctrl; 
    
    // Inner loops: PD ONLY (ki=0.0) to prevent windup
    PIDController att_roll_pid(5.0, 0.0, 1.0, -1.5, 1.5);
    PIDController att_pitch_pid(5.0, 0.0, 1.0, -1.5, 1.5);
    PIDController rate_roll_pid(0.1, 0.0, 0.003, -0.5, 0.5);
    PIDController rate_pitch_pid(0.1, 0.0, 0.003, -0.5, 0.5);
    PIDController alt_pid(1.5, 0.3, 0.3, -3.0, 3.0);
    
    std::ofstream file("phase3_waypoint_wind.csv");
    file << "time,target_x,actual_x,target_y,actual_y,z,wind_x\n";
    
    Eigen::Vector2d target_pos(0.0, 0.0);
    
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        
        if (t >= 2.0) target_pos = Eigen::Vector2d(2.0, 2.0);
        
        Eigen::Vector2d angles = pos_ctrl.compute(target_pos, state.position.head<2>(), state.velocity.head<2>(), dt);
        double thrust_adj = alt_pid.compute(5.0, state.position.z(), dt);
        double total_thrust = std::clamp(9.81 + thrust_adj, 0.0, 15.0);
        
        Eigen::Vector3d euler = state.attitude.toRotationMatrix().eulerAngles(2, 1, 0);
        double tr = att_roll_pid.compute(angles.x(), euler.z(), dt);
        double tp = att_pitch_pid.compute(angles.y(), euler.y(), dt);
        double tx = rate_roll_pid.compute(tr, state.angular_rates.x(), dt);
        double ty = rate_pitch_pid.compute(tp, state.angular_rates.y(), dt);
        
        Eigen::Vector4d forces = dynamics.getMixer().mix(total_thrust, Eigen::Vector3d(tx, ty, 0.0));
        
        Eigen::Vector3d wind = Eigen::Vector3d::Zero();
        if (t >= 4.0 && t <= 5.0) wind = Eigen::Vector3d(3.0, 0.0, 0.0);
        
        dynamics.step(state, forces, dt, wind);
        file << t << "," << target_pos.x() << "," << state.position.x() << ","
             << target_pos.y() << "," << state.position.y() << "," << state.position.z() << "," << wind.x() << "\n";
    }
    file.close();
    std::cout << "Done.\n";
    return 0;
}
