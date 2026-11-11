#pragma once
#include <Eigen/Dense>
#include <cmath>

class FaultDetector {
private:
    double threshold;
    Eigen::Vector3d inertia;
    Eigen::Vector3d prev_omega;
    bool is_fault;
    double fault_time;

public:
    FaultDetector(double thresh = 2.0, double Ixx = 0.01, double Iyy = 0.01, double Izz = 0.02)
        : threshold(thresh), inertia(Ixx, Iyy, Izz), prev_omega(0.0, 0.0, 0.0), is_fault(false), fault_time(-1.0) {}

    // Returns true if a fault is detected
    bool update(const Eigen::Vector3d& tau_cmd, const Eigen::Vector3d& omega_meas, double dt) {
        if (is_fault) return true; // Once in fault, stay in fault

        // 1. Calculate Measured Angular Acceleration (finite difference)
        Eigen::Vector3d omega_dot_meas = (omega_meas - prev_omega) / dt;
        prev_omega = omega_meas;

        // 2. Calculate Expected Angular Acceleration based on Commanded Torque
        // Euler's equation: tau = I * omega_dot + omega x (I * omega)
        // Therefore: omega_dot_exp = I^-1 * (tau_cmd - omega x (I * omega))
        Eigen::Vector3d gyroscopic_term = omega_meas.cross(inertia.cwiseProduct(omega_meas));
        Eigen::Vector3d omega_dot_exp = (tau_cmd - gyroscopic_term).cwiseQuotient(inertia);

        // 3. Calculate Residual (Norm of the difference)
        Eigen::Vector3d residual_vec = omega_dot_exp - omega_dot_meas;
        double residual_norm = residual_vec.norm();

        // 4. Check Threshold
        if (residual_norm > threshold) {
            is_fault = true;
            // fault_time will be set by the caller or we can track it here
        }

        return is_fault;
    }

    bool isFault() const { return is_fault; }
    void reset() { 
        is_fault = false; 
        prev_omega.setZero(); 
    }
};
