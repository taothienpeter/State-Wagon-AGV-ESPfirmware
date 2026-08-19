#include "trajectory_gen.h"
#include <math.h>
#include <string.h>

#define PI_F       3.14159265f
#define PI_OVER_2  1.57079633f
#define PI_OVER_3  1.04719755f

static float normalizeAngle(float a) {
    return atan2f(sinf(a), cosf(a));
}

TrajectoryGen::TrajectoryGen(const MotionProfile& profile)
    : active_index_(-1), current_speed_mps_(0),
      paused_(false), estop_(false),
      status_(TrajStatus::IDLE), profile_(profile)
{}

void TrajectoryGen::loadWaypoints(const std::vector<Waypoint>& wps) {
    waypoints_ = wps;
    active_index_ = waypoints_.empty() ? -1 : 0;
    current_speed_mps_ = 0;
    paused_ = false;
    estop_ = false;
    status_ = waypoints_.empty() ? TrajStatus::IDLE : TrajStatus::ACTIVE;
}

void TrajectoryGen::reset() {
    waypoints_.clear();
    active_index_ = -1;
    current_speed_mps_ = 0;
    paused_ = false;
    estop_ = false;
    status_ = TrajStatus::IDLE;
}

void TrajectoryGen::pause() {
    paused_ = true;
    status_ = TrajStatus::PAUSED;
}

void TrajectoryGen::resume() {
    paused_ = false;
    if (active_index_ >= 0 && active_index_ < (int)waypoints_.size())
        status_ = TrajStatus::ACTIVE;
    else
        status_ = TrajStatus::IDLE;
}

void TrajectoryGen::emergencyStop() {
    estop_ = true;
    status_ = TrajStatus::ESTOP;
    current_speed_mps_ = 0;
}

void TrajectoryGen::clearEstop() {
    estop_ = false;
    status_ = active_index_ >= 0 ? TrajStatus::ACTIVE : TrajStatus::IDLE;
}

float TrajectoryGen::computeProfileSpeed(float dist_remaining, float max_speed,
                                          float current_speed, float dt_s)
{
    float d_stop = (current_speed * current_speed) / (2.0f * profile_.max_decel_mps2);

    if (dist_remaining <= d_stop) {
        /* DECELERATE */
        float speed = sqrtf(2.0f * profile_.max_decel_mps2 * dist_remaining);
        return (speed < 0) ? 0 : speed;
    } else if (current_speed < max_speed) {
        /* ACCELERATE */
        float speed = current_speed + profile_.max_accel_mps2 * dt_s;
        if (speed < 0.0f)
            speed = 0.0f;
        return (speed > max_speed) ? max_speed : speed;
    } else {
        /* CRUISE */
        return max_speed;
    }
}

BodyVelocity TrajectoryGen::tick(const float pose[3], float dt_s) {
    BodyVelocity result = {0, 0};

    /* ---- Guard checks ---- */
    if (estop_) {
        current_speed_mps_ = 0;
        return result; /* zero velocity = safe stop */
    }
    if (paused_ || status_ == TrajStatus::IDLE || status_ == TrajStatus::COMPLETE || active_index_ < 0) {
        current_speed_mps_ = 0;
        return result;
    }
    if (active_index_ >= (int)waypoints_.size()) {
        status_ = TrajStatus::COMPLETE;
        current_speed_mps_ = 0;
        return result;
    }

    const Waypoint& wp = waypoints_[active_index_];

    /* ---- Compute position error ---- */
    float dx = wp.x - pose[0];
    float dy = wp.y - pose[1];
    float dist = sqrtf(dx * dx + dy * dy);

    /* ---- Compute heading error ---- */
    float target_heading = atan2f(dy, dx);
    float heading_error = normalizeAngle(target_heading - pose[2]);
    if (heading_error > PI_OVER_3)
        heading_error = PI_OVER_3;
    else if (heading_error < -PI_OVER_3)
        heading_error = -PI_OVER_3;

    /* ---- Speed profile (trapezoidal) ---- */
    float speed = computeProfileSpeed(dist, wp.max_speed_mps,
                                       current_speed_mps_, dt_s);
    current_speed_mps_ = speed;

    /* ---- Arrival detection ---- */
    if (dist < wp.tolerance_m) {
        active_index_++;
        if (active_index_ >= (int)waypoints_.size()) {
            status_ = TrajStatus::COMPLETE;
            current_speed_mps_ = 0;
            return result;
        }
        /* Start next segment immediately on same tick */
        return tick(pose, 0.0f);
    }

    /* ---- Build BodyVelocity ---- */
    result.vx_mps = speed;
    result.omega_radps = profile_.steering_gain * heading_error;

    return result;
}
