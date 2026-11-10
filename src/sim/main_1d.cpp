#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>

// --- 1. PID Controller ---
class PIDController {
private:
    double kp, ki, kd;
    double integral = 0.0;
    double prev_error = 0.0;

public:
    PIDController(double p, double i, double d) : kp(p), ki(i), kd(d) {}

    double compute(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return (kp * error) + (ki * integral) + (kd * derivative);
    }
    
    void reset() {
        integral = 0.0;
        prev_error = 0.0;
    }
};

// --- 2. 1D Physics Model (Semi-Implicit Euler) ---
class Drone1D {
private:
    double mass;
    double gravity;
    double drag_coeff; 
    
public:
    double z = 0.0;   // altitude (m)
    double vz = 0.0;  // velocity (m/s)
    double az = 0.0;  // acceleration (m/s^2)

    Drone1D(double m, double g, double drag) : mass(m), gravity(g), drag_coeff(drag) {}

    void update(double thrust, double dt) {
        // F_net = Thrust - mg - drag*v
        double F_net = thrust - (mass * gravity) - (drag_coeff * vz);
        az = F_net / mass;
        
        // Semi-implicit Euler integration
        vz += az * dt;
        z += vz * dt;
    }
};

int main() {
    // Simulation parameters
    double dt = 0.01;         // 10 ms time step (100 Hz)
    double total_time = 10.0; // 10 seconds
    int steps = total_time / dt;
    
    // Drone parameters (1kg class quadcopter)
    double mass = 1.0;
    double gravity = 9.81;
    double drag = 0.5;
    
    // Target altitude
    double target_z = 5.0; 
    
    // PID gains (Tune these to see how the drone reacts!)
    PIDController pid(8.0, 2.0, 1.5); 
    
    Drone1D drone(mass, gravity, drag);
    
    // CSV logging
    std::ofstream file("simulation_data.csv");
    file << "time,z,vz,az,thrust,error\n";
    
    std::cout << "Starting 1D Quadcopter Simulation...\n";
    
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double error = target_z - drone.z;
        
        // PID outputs thrust command
        double thrust = pid.compute(error, dt);
        
        // Physical constraints: Motors can only push, not pull
        if (thrust < 0.0) thrust = 0.0;
        if (thrust > 20.0) thrust = 20.0; // Max thrust limit
        
        // Update physics
        drone.update(thrust, dt);
        
        // Log data to CSV
        file << t << "," << drone.z << "," << drone.vz << "," 
             << drone.az << "," << thrust << "," << error << "\n";
    }
    
    file.close();
    std::cout << "Simulation complete. Data saved to simulation_data.csv\n";
    return 0;
}
