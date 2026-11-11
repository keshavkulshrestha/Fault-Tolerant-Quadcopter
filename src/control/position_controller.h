#pragma once
#include <Eigen/Dense>
#include <algorithm>

class PositionController {
public:
    Eigen::Vector2d compute(const Eigen::Vector2d& target, const Eigen::Vector2d& pos, 
                            const Eigen::Vector2d& vel, double dt) {
        double ex = target.x() - pos.x();
        double ey = target.y() - pos.y();
        
        // Extremely aggressive damping to force stopping
        double pitch_cmd = 0.5 * ex - 10.0 * vel.x();
        double roll_cmd = -0.5 * ey - 10.0 * vel.y();
        
        pitch_cmd = std::clamp(pitch_cmd, -0.15, 0.15);
        roll_cmd = std::clamp(roll_cmd, -0.15, 0.15);
        
        return Eigen::Vector2d(roll_cmd, pitch_cmd);
    }
};
