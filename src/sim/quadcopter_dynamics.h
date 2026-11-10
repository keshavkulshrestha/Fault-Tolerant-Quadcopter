#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "quadcopter_state.h"
#include "motor_mixer.h"

class QuadcopterDynamics {
private:
    double mass;
    double gravity;
    double drag_coeff;
    Eigen::Vector3d inertia; // Diagonal inertia tensor [Ixx, Iyy, Izz]
    MotorMixer mixer;

public:
    QuadcopterDynamics(double m = 1.0, double g = 9.81, double drag = 0.1, 
                       double Ixx = 0.01, double Iyy = 0.01, double Izz = 0.02)
        : mass(m), gravity(g), drag_coeff(drag), inertia(Ixx, Iyy, Izz), mixer(0.25, 0.05) {}

    // Update the state by one time step (dt in seconds)
    void step(QuadcopterState& state, const Eigen::Vector4d& motor_forces, double dt) {
        // 1. Calculate Total Thrust and Torques from motor forces
        // Using the forward mixing matrix logic
        double T = motor_forces.sum();
        
        // Torques based on our mixer definition:
        // Roll (x): L * (f2 - f4)
        // Pitch (y): L * (-f1 + f3)
        // Yaw (z): c * (-f1 + f2 - f3 + f4)
        double L = 0.25;
        double c = 0.05;
        Eigen::Vector3d tau;
        tau.x() = L * (motor_forces(1) - motor_forces(3));
        tau.y() = L * (-motor_forces(0) + motor_forces(2));
        tau.z() = c * (-motor_forces(0) + motor_forces(1) - motor_forces(2) + motor_forces(3));

        // 2. Rotation Matrix from Quaternion (Body to World)
        Eigen::Matrix3d R = state.attitude.toRotationMatrix();

        // 3. Translational Dynamics (Newton's 2nd Law in World Frame)
        // F_thrust is in body Z, so we rotate it to world frame: R * [0, 0, T]^T
        Eigen::Vector3d thrust_world = R * Eigen::Vector3d(0.0, 0.0, T);
        Eigen::Vector3d gravity_force = Eigen::Vector3d(0.0, 0.0, -mass * gravity);
        Eigen::Vector3d drag_force = -drag_coeff * state.velocity;
        
        Eigen::Vector3d acceleration = (thrust_world + gravity_force + drag_force) / mass;

        // 4. Rotational Dynamics (Euler's Rotation Equation in Body Frame)
        // I * alpha = tau - omega x (I * omega)
        Eigen::Vector3d angular_momentum = inertia.cwiseProduct(state.angular_rates);
        Eigen::Vector3d gyroscopic_torque = state.angular_rates.cross(angular_momentum);
        Eigen::Vector3d angular_acceleration = (tau - gyroscopic_torque).cwiseQuotient(inertia);

        // 5. Semi-Implicit Euler Integration
        // Update velocities first, then positions/orientation
        state.velocity += acceleration * dt;
        state.position += state.velocity * dt;

        state.angular_rates += angular_acceleration * dt;
        
        // Quaternion derivative: q_dot = 0.5 * q * omega_quat
        Eigen::Quaterniond omega_quat(0.0, state.angular_rates.x(), state.angular_rates.y(), state.angular_rates.z());
        Eigen::Quaterniond q_dot = Eigen::Quaterniond(0.5 * (state.attitude * omega_quat).coeffs());
        
        state.attitude.coeffs() += q_dot.coeffs() * dt;
        state.attitude.normalize(); // Crucial: keep quaternion normalized!
    }

    const MotorMixer& getMixer() const { return mixer; }
};
