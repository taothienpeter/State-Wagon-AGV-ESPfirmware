#ifndef TRAJECTORY_GEN_H
#define TRAJECTORY_GEN_H

#include <stdint.h>
#include <vector>
#include "swerve_kinematics.h"

#ifdef __cplusplus

struct Waypoint {
    float x, y;
    float max_speed_mps = 1.0f;
    float tolerance_m = 0.05f;
};

struct MotionProfile {
    float max_accel_mps2 = 0.5f;
    float max_decel_mps2 = 0.8f;
    float steering_gain = 1.0f;
};

enum class TrajStatus : uint8_t {
    IDLE,
    ACTIVE,
    PAUSED,
    COMPLETE,
    ESTOP
};

class TrajectoryGen {
public:
    explicit TrajectoryGen(const MotionProfile& profile = MotionProfile{});

    /* Load flat waypoint list from server order */
    void loadWaypoints(const std::vector<Waypoint>& wps);

    /* Reset to idle state */
    void reset();

    /* Pause / resume / emergency stop */
    void pause();
    void resume();
    void emergencyStop();
    void clearEstop();

    /* Main tick: returns body velocity command for current tick.
       pose = [x, y, theta] from EKF.
       dt_s = time since last tick. */
    BodyVelocity tick(const float pose[3], float dt_s);

    /* State accessors */
    TrajStatus status() const { return status_; }
    float currentSpeedMps() const { return current_speed_mps_; }
    int activeIndex() const { return active_index_; }

private:
    /* Compute trapezoidal speed profile */
    float computeProfileSpeed(float dist_remaining, float max_speed,
                               float current_speed, float dt_s);

    std::vector<Waypoint> waypoints_;
    int active_index_;
    float current_speed_mps_;
    bool paused_;
    bool estop_;
    TrajStatus status_;
    MotionProfile profile_;
};

#endif /* __cplusplus */
#endif /* TRAJECTORY_GEN_H */
