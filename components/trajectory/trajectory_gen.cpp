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
#if defined(ESP_PLATFORM)
      , mux_(nullptr)
#endif
{
#if defined(ESP_PLATFORM)
    mux_ = xSemaphoreCreateRecursiveMutex();
#endif
}

TrajectoryGen::~TrajectoryGen() {
#if defined(ESP_PLATFORM)
    if (mux_) {
        vSemaphoreDelete(mux_);
        mux_ = nullptr;
    }
#endif
}

void TrajectoryGen::lock() const {
#if defined(ESP_PLATFORM)
    if (mux_) {
        xSemaphoreTakeRecursive(mux_, portMAX_DELAY);
    }
#endif
}

void TrajectoryGen::unlock() const {
#if defined(ESP_PLATFORM)
    if (mux_) {
        xSemaphoreGiveRecursive(mux_);
    }
#endif
}

void TrajectoryGen::loadWaypoints(const std::vector<Waypoint>& wps) {
    lock();
    waypoints_ = wps;
    active_index_ = waypoints_.empty() ? -1 : 0;
    current_speed_mps_ = 0;
    paused_ = false;
    estop_ = false;
    status_ = waypoints_.empty() ? TrajStatus::IDLE : TrajStatus::ACTIVE;
    unlock();
}

void TrajectoryGen::reset() {
    lock();
    waypoints_.clear();
    active_index_ = -1;
    current_speed_mps_ = 0;
    paused_ = false;
    estop_ = false;
    status_ = TrajStatus::IDLE;
    unlock();
}

void TrajectoryGen::pause() {
    lock();
    paused_ = true;
    status_ = TrajStatus::PAUSED;
    unlock();
}

void TrajectoryGen::resume() {
    lock();
    paused_ = false;
    if (active_index_ >= 0 && active_index_ < (int)waypoints_.size())
        status_ = TrajStatus::ACTIVE;
    else
        status_ = TrajStatus::IDLE;
    unlock();
}

void TrajectoryGen::emergencyStop() {
    lock();
    estop_ = true;
    status_ = TrajStatus::ESTOP;
    current_speed_mps_ = 0;
    unlock();
}

void TrajectoryGen::clearEstop() {
    lock();
    estop_ = false;
    status_ = active_index_ >= 0 ? TrajStatus::ACTIVE : TrajStatus::IDLE;
    unlock();
}

TrajStatus TrajectoryGen::status() const {
    lock();
    TrajStatus s = status_;
    unlock();
    return s;
}

const char* TrajectoryGen::orderStateString() const {
    lock();
    TrajStatus s = status_;
    unlock();

    switch (s) {
    case TrajStatus::IDLE:     return "IDLE";
    case TrajStatus::ACTIVE:   return "ACTIVE";
    case TrajStatus::PAUSED:   return "PAUSED";
    case TrajStatus::COMPLETE: return "COMPLETED";
    case TrajStatus::ESTOP:    return "ESTOP";
    default:                   return "IDLE";
    }
}

float TrajectoryGen::currentSpeedMps() const {
    lock();
    float spd = current_speed_mps_;
    unlock();
    return spd;
}

int TrajectoryGen::activeIndex() const {
    lock();
    int idx = active_index_;
    unlock();
    return idx;
}

float TrajectoryGen::computeProfileSpeed(float dist_remaining, float max_speed,
                                          float current_speed, float dt_s)
{
    float d_stop = (current_speed * current_speed) / (2.0f * profile_.max_decel_mps2);

    if (dist_remaining <= d_stop) {
        /* DECELERATE */
        float safe_dist = fmaxf(0.0f, dist_remaining);
        float speed = sqrtf(2.0f * profile_.max_decel_mps2 * safe_dist);
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
    lock();
    BodyVelocity result = {0, 0};

    /* ---- Guard checks ---- */
    if (estop_) {
        current_speed_mps_ = 0;
        unlock();
        return result; /* zero velocity = safe stop */
    }
    if (paused_ || status_ == TrajStatus::IDLE || status_ == TrajStatus::COMPLETE || active_index_ < 0) {
        current_speed_mps_ = 0;
        unlock();
        return result;
    }
    if (active_index_ >= (int)waypoints_.size()) {
        status_ = TrajStatus::COMPLETE;
        current_speed_mps_ = 0;
        unlock();
        return result;
    }

    const Waypoint& wp = waypoints_[active_index_];

    /* ---- 1. World Frame Position Error ---- */
    float dx = wp.x - pose[0];
    float dy = wp.y - pose[1];
    float dist = sqrtf(dx * dx + dy * dy);

    /* ---- 2. Chassis Heading Error (Independent of Motion Vector) ---- */
    float heading_error = 0.0f;
    if (wp.has_theta) {
        heading_error = normalizeAngle(wp.theta_rad - pose[2]);
    } else {
        /* When no target theta is specified, hold current chassis heading (swerve strafe) */
        heading_error = 0.0f;
    }

    /* ---- 3. Linear Speed Profile (Trapezoidal along path vector) ---- */
    float speed = computeProfileSpeed(dist, wp.max_speed_mps,
                                       current_speed_mps_, dt_s);
    current_speed_mps_ = speed;

    /* ---- 4. Arrival Detection (Position AND Chassis Heading) ---- */
    bool is_last_wp = (active_index_ == (int)waypoints_.size() - 1);
    float effective_tol = is_last_wp ? wp.tolerance_m : fmaxf(wp.tolerance_m, 0.15f);

    bool pos_arrived = (dist < effective_tol);
    bool heading_arrived = (!wp.has_theta || fabsf(heading_error) < wp.tolerance_theta_rad);

    if (pos_arrived && heading_arrived) {
        active_index_++;
        if (active_index_ >= (int)waypoints_.size()) {
            status_ = TrajStatus::COMPLETE;
            current_speed_mps_ = 0;
            unlock();
            return result;
        }
        /* Start next segment immediately on same tick */
        unlock();
        return tick(pose, 0.0f);
    }

    /* ---- 5. Holonomic Swerve Drive Velocity Computation ---- */
    // World Frame Linear Velocity Vector (directed from current pos to waypoint):
    float vw_x = 0.0f;
    float vw_y = 0.0f;
    if (dist > 0.001f) {
        vw_x = speed * (dx / dist);
        vw_y = speed * (dy / dist);
    }

    // Transform World Velocity Vector to Robot Chassis Body Frame:
    // vx_body =  vw_x * cos(theta) + vw_y * sin(theta)
    // vy_body = -vw_x * sin(theta) + vw_y * cos(theta)
    float current_theta = pose[2];
    float cos_t = cosf(current_theta);
    float sin_t = sinf(current_theta);

    result.vx_mps =  vw_x * cos_t + vw_y * sin_t;
    result.vy_mps = -vw_x * sin_t + vw_y * cos_t;

    // Independent Chassis Rotational Velocity (Omega):
    if (wp.has_theta) {
        float max_w = (wp.max_omega_radps > 0.1f) ? wp.max_omega_radps : 2.0f;
        float omega_cmd = profile_.steering_gain * 2.0f * heading_error;
        if (omega_cmd > max_w) omega_cmd = max_w;
        else if (omega_cmd < -max_w) omega_cmd = -max_w;
        result.omega_radps = omega_cmd;
    } else {
        result.omega_radps = 0.0f;
    }

    unlock();
    return result;
}

