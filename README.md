# Fault-Tolerant Quadcopter Flight Controller

A C++ simulation of a quadcopter that detects a failed motor in flight and recovers, instead of crashing.

## 🚀 Project Overview
This project builds a 6-DOF rigid-body quadcopter simulation from scratch. It features cascaded PID control, sensor fusion, and a fault-tolerant controller that allows the drone to safely land even if a motor fails mid-flight.

## 🏗️ Architecture
- **`src/sim/`**: Physics engine, motor models, and wind disturbances.
- **`src/control/`**: Cascaded PID loops (Rate, Attitude, Position) and motor mixers.
- **`src/estimation/`**: IMU sensor models and Complementary/EKF filters.
- **`src/fdi/`**: Fault Detection and Isolation (residual-based).
- **`src/supervisor/`**: State machine (NORMAL, FAULT, RECOVERY, LANDED).
- **`tools/`**: Python scripts for plotting and 3D animation.

## 📊 Current Status
🚧 **Phase 0: Setup and C++ Foundations** - *In Progress*
- [x] Repository initialized
- [x] Folder structure created
- [ ] 1D Altitude PID Simulation (Milestone 0)

## 🛠️ Tech Stack
- **Language**: C++17
- **Math/Physics**: Eigen (Linear Algebra), Semi-Implicit Euler / RK4 Integration
- **Build System**: CMake
- **Visualization**: Python (Matplotlib)
