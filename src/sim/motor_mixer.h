#pragma once
#include <Eigen/Dense>
#include <iostream>

class MotorMixer {
private:
    double arm_length;
    double drag_to_thrust_ratio;
    Eigen::Matrix4d mixing_matrix;
    Eigen::Matrix4d inverse_mixing_matrix;

public:
    MotorMixer(double L = 0.25, double c = 0.05) 
        : arm_length(L), drag_to_thrust_ratio(c) {
        
        // Build the mixing matrix for an "X" configuration
        // Motors: 1=Front-Right, 2=Front-Left, 3=Back-Left, 4=Back-Right
        mixing_matrix << 
            1.0,  1.0,  1.0,  1.0,
            0.0,  L,    0.0, -L,
           -L,    0.0,  L,    0.0,
           -c,    c,   -c,    c;
            
        // Pre-compute the inverse so we can go from [T, tx, ty, tz] -> [f1, f2, f3, f4]
        inverse_mixing_matrix = mixing_matrix.inverse();
    }

    // Convert desired thrust and torques to individual motor forces
    Eigen::Vector4d mix(double total_thrust, const Eigen::Vector3d& torques) const {
        Eigen::Vector4d commands;
        commands << total_thrust, torques.x(), torques.y(), torques.z();
        
        Eigen::Vector4d motor_forces = inverse_mixing_matrix * commands;
        
        // Physical constraint: Motors can only push (force >= 0)
        return motor_forces.cwiseMax(0.0);
    }
};
