#ifndef SWERVE_KINEMATICS_H
#define SWERVE_KINEMATICS_H

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus

#define SWERVE_EPSILON 0.01f

struct SwerveCommand {
    float steering_angle_rad = 0.0f;
    float drive_velocity_mps = 0.0f;
    float steering_rate_radps = 0.0f;
};

struct VehicleGeometry {
    float wheelbase_m = 0.42f;
    float wheel_radius_m = 0.075f;
    float max_velocity_mps = 2.0f;
    float max_steering_angle_rad = 1.57f;
    float max_steering_rate_radps = 6.0f;
};

struct BodyVelocity {
    float vx_mps;
    float omega_radps;
};

class SwerveKinematics {
public:
    SwerveKinematics() {}
    explicit SwerveKinematics(const VehicleGeometry& geom) : geom_(geom) {
        prev_cmd_[0] = prev_cmd_[1] = {0, 0, 0};
        last_cmd_dt_ = 0.01f;
    }

    void setGeometry(const VehicleGeometry& geom) { geom_ = geom; }
    const VehicleGeometry& geometry() const { return geom_; }

    /* Body velocity → per-wheel commands (index 0=front, 1=rear).
       Returns false if both vx and omega are effectively zero (idle). */
    bool toSwerveCommand(const BodyVelocity& cmd, float dt_s, SwerveCommand wheels[2]) {
        float half_L = geom_.wheelbase_m * 0.5f;
        float vx = cmd.vx_mps;
        float omega = cmd.omega_radps;

        if (fabsf(vx) < SWERVE_EPSILON && fabsf(omega) < SWERVE_EPSILON) {
            wheels[0] = prev_cmd_[0];
            wheels[1] = prev_cmd_[1];
            wheels[0].drive_velocity_mps = 0;
            wheels[1].drive_velocity_mps = 0;
            return false;
        }

        float dt = dt_s;
        if (dt < 0.001f) dt = last_cmd_dt_;
        last_cmd_dt_ = dt;

        for (int i = 0; i < 2; i++) {
            float x_i = (i == 0) ? half_L : -half_L;
            float v_i_y = omega * x_i;

            float angle, speed;
            if (fabsf(vx) > SWERVE_EPSILON) {
                angle = atan2f(v_i_y, vx);
                speed = sqrtf(vx * vx + v_i_y * v_i_y);
            } else {
                /* Pure rotation: wheels turn 90° to rotate in place */
                angle = (omega > 0) ? (float)M_PI_2 : -(float)M_PI_2;
                speed = fabsf(v_i_y);
            }

            /* Apply limits */
            if (angle > geom_.max_steering_angle_rad)
                angle = geom_.max_steering_angle_rad;
            else if (angle < -geom_.max_steering_angle_rad)
                angle = -geom_.max_steering_angle_rad;

            if (speed > geom_.max_velocity_mps)
                speed = geom_.max_velocity_mps;

            /* Steering rate limit */
            float max_delta = geom_.max_steering_rate_radps * dt;
            float prev_angle = prev_cmd_[i].steering_angle_rad;
            if (angle > prev_angle + max_delta)
                angle = prev_angle + max_delta;
            else if (angle < prev_angle - max_delta)
                angle = prev_angle - max_delta;

            /* Deadband */
            if (speed < 0.01f) {
                speed = 0;
                angle = prev_angle;
            }

            wheels[i].steering_angle_rad = angle;
            wheels[i].drive_velocity_mps = speed;
            wheels[i].steering_rate_radps = geom_.max_steering_rate_radps;

            prev_cmd_[i] = wheels[i];
        }
        return true;
    }

    /* Inverse: wheel feedback → body velocity estimate (for odometry/EKF) */
    BodyVelocity fromWheelFeedback(const float steering_angles[2], const float drive_speeds[2]) {
        float half_L = geom_.wheelbase_m * 0.5f;
        BodyVelocity result = {0, 0};
        int count = 0;

        for (int i = 0; i < 2; i++) {
            float angle = steering_angles[i];
            float speed = drive_speeds[i];
            float x_i = (i == 0) ? half_L : -half_L;

            float vx_i = speed * cosf(angle);
            float omega_i;
            if (fabsf(x_i) > SWERVE_EPSILON)
                omega_i = speed * sinf(angle) / x_i;
            else
                omega_i = 0;

            result.vx_mps += vx_i;
            result.omega_radps += omega_i;
            count++;
        }

        if (count > 0) {
            result.vx_mps /= count;
            result.omega_radps /= count;
        }
        return result;
    }

private:
    VehicleGeometry geom_;
    SwerveCommand prev_cmd_[2];
    float last_cmd_dt_;
};

#endif /* __cplusplus */
#endif /* SWERVE_KINEMATICS_H */
