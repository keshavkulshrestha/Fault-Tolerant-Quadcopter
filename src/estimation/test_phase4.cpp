#include <iostream>
#include <fstream>
#include <cmath>
#include "imu_sensor.h"
#include "complementary_filter.h"

int main() {
    std::cout << "--- Phase 4: IMU Noise & Complementary Filter ---\n";
    
    double dt = 0.01; // 100 Hz sensor rate
    double total_time = 10.0;
    int steps = total_time / dt;
    
    IMUSensor imu(0.8, 0.05); // 0.8 m/s^2 accel noise, 0.05 rad/s gyro noise
    ComplementaryFilter filter(0.98); // Trust gyro 98%, accel 2%
    
    std::ofstream file("phase4_estimation.csv");
    file << "time,true_roll,noisy_accel_roll,noisy_gyro_roll,filtered_roll\n";
    
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        
        // 1. Simulate a "True" maneuver: A smooth 20-degree (0.35 rad) roll sine wave
        double true_roll = 0.35 * std::sin(2.0 * M_PI * 0.2 * t); 
        double true_roll_rate = 0.35 * 2.0 * M_PI * 0.2 * std::cos(2.0 * M_PI * 0.2 * t);
        
        // Simulate true accelerometer reading (gravity projected onto axes)
        Eigen::Vector3d true_accel;
        true_accel.x() = -9.81 * std::sin(true_roll);
        true_accel.y() = 0.0;
        true_accel.z() = 9.81 * std::cos(true_roll);
        
        Eigen::Vector3d true_gyro(true_roll_rate, 0.0, 0.0);
        
        // 2. Get Noisy Sensor Data
        Eigen::Vector3d noisy_accel, noisy_gyro;
        imu.measure(true_accel, true_gyro, noisy_accel, noisy_gyro);
        
        // Calculate raw noisy angles for plotting
        double noisy_accel_roll = std::atan2(-noisy_accel.x(), noisy_accel.z());
        double noisy_gyro_roll = true_roll_rate; // Just plotting the rate for reference, or we could integrate it raw
        
        // 3. Run Complementary Filter
        filter.update(noisy_accel, noisy_gyro, dt);
        double filtered_roll = filter.getRoll();
        
        // Log to CSV
        file << t << "," << true_roll << "," << noisy_accel_roll << "," << noisy_gyro.x() << "," << filtered_roll << "\n";
    }
    
    file.close();
    std::cout << "Simulation complete. Data saved to phase4_estimation.csv\n";
    return 0;
}
