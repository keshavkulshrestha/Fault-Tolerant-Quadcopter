#pragma once
#include <Eigen/Dense>
#include <random>

class IMUSensor {
private:
    std::mt19937 generator;
    std::normal_distribution<double> accel_noise;
    std::normal_distribution<double> gyro_noise;

public:
    // Constructor with noise standard deviations
    IMUSensor(double accel_std = 0.5, double gyro_std = 0.05) 
        : generator(42), // Fixed seed for reproducible results
          accel_noise(0.0, accel_std), 
          gyro_noise(0.0, gyro_std) {}

    // Takes true acceleration and angular rates, returns noisy measurements
    void measure(const Eigen::Vector3d& true_accel, const Eigen::Vector3d& true_gyro,
                 Eigen::Vector3d& noisy_accel, Eigen::Vector3d& noisy_gyro) {
        
        noisy_accel.x() = true_accel.x() + accel_noise(generator);
        noisy_accel.y() = true_accel.y() + accel_noise(generator);
        noisy_accel.z() = true_accel.z() + accel_noise(generator);

        noisy_gyro.x() = true_gyro.x() + gyro_noise(generator);
        noisy_gyro.y() = true_gyro.y() + gyro_noise(generator);
        noisy_gyro.z() = true_gyro.z() + gyro_noise(generator);
    }
};
