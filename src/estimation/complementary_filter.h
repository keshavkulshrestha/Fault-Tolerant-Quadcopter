#pragma once
#include <Eigen/Dense>
#include <cmath>

class ComplementaryFilter {
private:
    double alpha; // Weight for gyroscope (usually 0.95 - 0.99)
    double roll_est, pitch_est;

public:
    ComplementaryFilter(double a = 0.98) : alpha(a), roll_est(0.0), pitch_est(0.0) {}

    // Update the filter with new noisy sensor data
    void update(const Eigen::Vector3d& noisy_accel, const Eigen::Vector3d& noisy_gyro, double dt) {
        // 1. Calculate angle from Accelerometer (using gravity)
        // atan2(-ax, az) gives roll, atan2(-ay, az) gives pitch
        double accel_roll = std::atan2(-noisy_accel.x(), noisy_accel.z());
        double accel_pitch = std::atan2(-noisy_accel.y(), noisy_accel.z());

        // 2. Calculate angle from Gyroscope (integrating rate)
        double gyro_roll = roll_est + noisy_gyro.x() * dt;
        double gyro_pitch = pitch_est + noisy_gyro.y() * dt;

        // 3. Fuse them! 
        // Trust gyro for fast changes, accel for long-term drift correction
        roll_est = alpha * gyro_roll + (1.0 - alpha) * accel_roll;
        pitch_est = alpha * gyro_pitch + (1.0 - alpha) * accel_pitch;
    }

    double getRoll() const { return roll_est; }
    double getPitch() const { return pitch_est; }
};
