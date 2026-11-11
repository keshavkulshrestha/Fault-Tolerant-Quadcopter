#pragma once

enum class FlightState {
    NORMAL,
    RECOVERY,
    LANDED
};

class Supervisor {
private:
    FlightState state;
    double target_altitude;
    double descent_rate;

public:
    Supervisor(double initial_alt = 5.0, double desc_rate = 1.0) 
        : state(FlightState::NORMAL), target_altitude(initial_alt), descent_rate(desc_rate) {}

    FlightState getState() const { return state; }
    double getTargetAltitude() const { return target_altitude; }

    void triggerRecovery() {
        if (state == FlightState::NORMAL) {
            state = FlightState::RECOVERY;
        }
    }

    void update(double dt, double current_altitude) {
        if (state == FlightState::RECOVERY) {
            // Command a slow, controlled descent
            target_altitude -= descent_rate * dt;
            
            // Check if we've reached the ground
            if (current_altitude <= 0.1) {
                state = FlightState::LANDED;
                target_altitude = 0.0;
            }
        }
    }

    void reset(double initial_alt = 5.0) {
        state = FlightState::NORMAL;
        target_altitude = initial_alt;
    }
};
