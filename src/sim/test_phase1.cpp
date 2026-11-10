#include <iostream>
#include <fstream>
#include "quadcopter_state.h"
#include "quadcopter_dynamics.h"

int main() {
    std::cout << "--- Phase 1: 6-DOF Dynamics Test ---\n";
    
    double dt = 0.01; 
    double total_time = 4.0; 
    int steps = total_time / dt;
    
    QuadcopterState state;
    state.position = Eigen::Vector3d(0.0, 0.0, 5.0); 
    
    QuadcopterDynamics dynamics(1.0, 9.81, 0.1, 0.01, 0.01, 0.02);
    
    std::ofstream file("phase1_dynamics.csv");
    // Header: time, x, y, z, vx, vy, vz, yaw, pitch, roll, T
    file << "time,x,y,z,vx,vy,vz,yaw,pitch,roll,T\n";
    
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        Eigen::Vector4d motor_forces = Eigen::Vector4d::Zero();
        
        // SCENARIO LOGIC:
        if (t < 2.0) {
            // 0.0 to 2.0s: PERFECT HOVER
            motor_forces = Eigen::Vector4d::Constant(9.81 / 4.0);
        } else if (t < 3.0) {
            // 2.0s to 3.0s: YAW SPIN
            // Keep total thrust = 9.81 N.
            // Increase diagonal motors (2 and 4) to create positive yaw torque.
            // F1=1.455, F2=3.455, F3=1.455, F4=3.455 => Sum = 9.81
            motor_forces << 1.455, 3.455, 1.455, 3.455;
        } else {
            // 3.0s to 4.0s: RETURN TO HOVER
            motor_forces = Eigen::Vector4d::Constant(9.81 / 4.0);
        }
        
        // Update physics
        dynamics.step(state, motor_forces, dt);
        
        // Extract Euler angles. eulerAngles(2, 1, 0) returns [yaw, pitch, roll]
        Eigen::Vector3d euler = state.attitude.toRotationMatrix().eulerAngles(2, 1, 0);
        
        double total_thrust = motor_forces.sum();
        
        // Log to CSV
        file << t << "," 
             << state.position.x() << "," << state.position.y() << "," << state.position.z() << ","
             << state.velocity.x() << "," << state.velocity.y() << "," << state.velocity.z() << ","
             << euler.x() << "," << euler.y() << "," << euler.z() << "," // yaw, pitch, roll
             << total_thrust << "\n";
    }
    
    file.close();
    std::cout << "Simulation complete. Data saved to phase1_dynamics.csv\n";
    std::cout << "Final Altitude: " << state.position.z() << " m (Expected: ~5.0 m)\n";
    std::cout << "Final Yaw: " << state.attitude.toRotationMatrix().eulerAngles(2, 1, 0).x() << " rad\n";
    
    return 0;
}
