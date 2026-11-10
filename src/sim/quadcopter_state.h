#pragma once
#include <Eigen/Dense>

// Represents the full 6-DOF state of the quadcopter
struct QuadcopterState {
    // Position in world frame [x, y, z] (meters)
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    
    // Velocity in world frame [vx, vy, vz] (m/s)
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    
    // Attitude as a Quaternion [w, x, y, z] (unit quaternion)
    // Initialized to identity (no rotation, flat on the ground)
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
    
    // Angular rates in body frame [p, q, r] (rad/s)
    Eigen::Vector3d angular_rates = Eigen::Vector3d::Zero();

    // Helper to print the state (useful for debugging)
    void print() const {
        std::cout << "Pos: [" << position.transpose() << "] | "
                  << "Vel: [" << velocity.transpose() << "] | "
                  << "Rates: [" << angular_rates.transpose() << "]\n";
    }
};
