# Fault-Tolerant Quadcopter Flight Controller

A C++ simulation of a quadcopter that detects a failed motor in flight and recovers, instead of crashing.

## 🚀 Project Overview
This project builds a 6-DOF rigid-body quadcopter simulation from scratch. It features cascaded PID control, sensor fusion, and a fault-tolerant controller that allows the drone to safely land even if a motor fails mid-flight.

## 📊 Current Status

✅ **Phase 0: Setup and C++ Foundations** — *Complete*
- [x] Repository initialized and WSL2 environment configured
- [x] 1D Altitude PID Simulation

✅ **Phase 1: Rigid-body quadcopter model** — *Complete*
- [x] 6-DOF dynamics (Newton-Euler equations)
- [x] Quaternion integration for drift-free attitude
- [x] Motor mixer and thrust/torque mapping
- [x] Validated: Free fall, perfect hover, and constant yaw spin

![Phase 1 3D Trajectory](docs/phase1_3d_trajectory.png)
*Figure 1: 3D flight path showing perfect 5m hover and final yaw orientation.*

🚧 **Phase 2: Inner loops (Rate & Attitude PID)** — *Next*

## 🏗️ Architecture
- **`src/sim/`**: Physics engine, motor models, and wind disturbances.
- **`src/control/`**: Cascaded PID loops and motor mixers.
- **`src/estimation/`**: IMU sensor models and Complementary/EKF filters.
- **`src/fdi/`**: Fault Detection and Isolation (residual-based).
- **`src/supervisor/`**: State machine (NORMAL, FAULT, RECOVERY, LANDED).

## 🛠️ Tech Stack
- **Language**: C++17
- **Math/Physics**: Eigen (Linear Algebra), Quaternions, Semi-Implicit Euler Integration
- **Build System**: CMake
- **Visualization**: Python (Matplotlib 3D)
