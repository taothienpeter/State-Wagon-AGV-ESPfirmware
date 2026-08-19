#define _USE_MATH_DEFINES
#include "catch.hpp"
#include "swerve_kinematics.h"
#include <cmath>

namespace {
constexpr float kHalfL = 0.21f;        /* wheelbase 0.42 / 2 */
constexpr float kMaxSteerRate = 3.14f; /* firmware max_steering_rate_radps */
}

struct TestFixture {
    VehicleGeometry fw_geom{0.42f, 0.075f, 1.2f, 1.57f, 3.14f};

    static BodyVelocity cmd(float vx, float omega) {
        return BodyVelocity{vx, omega};
    }
};

TEST_CASE_METHOD(TestFixture, "Default-constructed instance is safe and idle", "[kinematics][state]") {
    SwerveKinematics kin;
    SwerveCommand wheels[2] = {{9.0f, 9.0f, 9.0f}, {9.0f, 9.0f, 9.0f}};
    REQUIRE_FALSE(kin.toSwerveCommand(cmd(0.0f, 0.0f), 0.02f, wheels));
    REQUIRE(wheels[0].drive_velocity_mps == Approx(0.0f));
    REQUIRE(wheels[1].drive_velocity_mps == Approx(0.0f));
    REQUIRE(wheels[0].steering_angle_rad == Approx(0.0f));
    REQUIRE(wheels[1].steering_angle_rad == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Sub-ms dt falls back to previous tick duration", "[kinematics][state]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    REQUIRE(kin.toSwerveCommand(cmd(1.0f, 2.0f), 0.0005f, wheels));
    float target = std::atan2(2.0f * kHalfL, 1.0f);
    REQUIRE(target > kMaxSteerRate * 0.01f); /* guaranteed rate-limited */
    REQUIRE(wheels[0].steering_angle_rad == Approx(kMaxSteerRate * 0.01f));
    REQUIRE(wheels[1].steering_angle_rad == Approx(-kMaxSteerRate * 0.01f));
}

TEST_CASE_METHOD(TestFixture, "Geometry accessors and setGeometry", "[kinematics][state]") {
    SwerveKinematics kin{fw_geom};
    REQUIRE(kin.geometry().wheelbase_m == Approx(0.42f));
    REQUIRE(kin.geometry().wheel_radius_m == Approx(0.075f));
    REQUIRE(kin.geometry().max_velocity_mps == Approx(1.2f));
    REQUIRE(kin.geometry().max_steering_angle_rad == Approx(1.57f));
    REQUIRE(kin.geometry().max_steering_rate_radps == Approx(3.14f));

    VehicleGeometry g = fw_geom;
    g.max_velocity_mps = 0.5f;
    kin.setGeometry(g);
    SwerveCommand wheels[2];
    REQUIRE(kin.toSwerveCommand(cmd(2.0f, 0.0f), 1.0f, wheels));
    REQUIRE(wheels[0].drive_velocity_mps == Approx(0.5f));
}

TEST_CASE_METHOD(TestFixture, "Idle command holds angle, zeroes speed, returns false", "[kinematics][forward]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    REQUIRE(kin.toSwerveCommand(cmd(1.0f, 0.0f), 1.0f, wheels));
    REQUIRE(wheels[0].steering_angle_rad == Approx(0.0f));

    REQUIRE_FALSE(kin.toSwerveCommand(cmd(0.0f, 0.0f), 1.0f, wheels));
    REQUIRE(wheels[0].steering_angle_rad == Approx(0.0f));
    REQUIRE(wheels[1].steering_angle_rad == Approx(0.0f));
    REQUIRE(wheels[0].drive_velocity_mps == Approx(0.0f));
    REQUIRE(wheels[1].drive_velocity_mps == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Straight motion: zero angle, full speed", "[kinematics][forward]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    REQUIRE(kin.toSwerveCommand(cmd(1.0f, 0.0f), 1.0f, wheels));
    REQUIRE(wheels[0].steering_angle_rad == Approx(0.0f));
    REQUIRE(wheels[1].steering_angle_rad == Approx(0.0f));
    REQUIRE(wheels[0].drive_velocity_mps == Approx(1.0f));
    REQUIRE(wheels[1].drive_velocity_mps == Approx(1.0f));
}

TEST_CASE_METHOD(TestFixture, "Turn splits wheels symmetrically by atan2 formula", "[kinematics][forward]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    REQUIRE(kin.toSwerveCommand(cmd(1.0f, 0.5f), 1.0f, wheels));

    float v_iy = 0.5f * kHalfL;
    float exp_angle = std::atan2(v_iy, 1.0f);
    float exp_speed = std::sqrt(1.0f + v_iy * v_iy);
    REQUIRE(wheels[0].steering_angle_rad == Approx(exp_angle));
    REQUIRE(wheels[1].steering_angle_rad == Approx(-exp_angle));
    REQUIRE(wheels[0].drive_velocity_mps == Approx(exp_speed));
    REQUIRE(wheels[1].drive_velocity_mps == Approx(exp_speed));
}

TEST_CASE_METHOD(TestFixture, "Positive omega steers front positive, rear negative", "[kinematics][forward]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    REQUIRE(kin.toSwerveCommand(cmd(1.0f, 0.5f), 1.0f, wheels));
    REQUIRE(wheels[0].steering_angle_rad > 0.0f);
    REQUIRE(wheels[1].steering_angle_rad < 0.0f);
}

TEST_CASE_METHOD(TestFixture, "Pure rotation: wheels at steering limit, v = omega*halfL", "[kinematics][forward]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    REQUIRE(kin.toSwerveCommand(cmd(0.0f, 1.0f), 1.0f, wheels));
    REQUIRE(wheels[0].steering_angle_rad == Approx(1.57f));
    REQUIRE(wheels[1].steering_angle_rad == Approx(-1.57f));
    REQUIRE(wheels[0].drive_velocity_mps == Approx(kHalfL));
    REQUIRE(wheels[1].drive_velocity_mps == Approx(kHalfL));
}

TEST_CASE_METHOD(TestFixture, "Drive speed clamps to max_velocity", "[kinematics][forward]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    REQUIRE(kin.toSwerveCommand(cmd(2.0f, 0.0f), 1.0f, wheels));
    REQUIRE(wheels[0].drive_velocity_mps == Approx(1.2f));
    REQUIRE(wheels[0].steering_angle_rad == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Reverse vx<0 maps to 90-degree steer (documented gap)", "[kinematics][forward][gap]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    REQUIRE(kin.toSwerveCommand(cmd(-1.0f, 0.0f), 1.0f, wheels));
    REQUIRE(wheels[0].steering_angle_rad == Approx(1.57f));
    REQUIRE(wheels[1].steering_angle_rad == Approx(-1.57f));
    REQUIRE(wheels[0].drive_velocity_mps == Approx(1.0f));
    REQUIRE(wheels[1].drive_velocity_mps == Approx(1.0f));
}

TEST_CASE_METHOD(TestFixture, "Steering rate limit bounds per-tick angle change", "[kinematics][dynamics]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    float target = std::atan2(2.0f * kHalfL, 1.0f);
    float max_delta = kMaxSteerRate * 0.02f;
    float prev_angle = 0.0f;
    float angle = 0.0f;
    for (int t = 0; t < 200 && angle < target; t++) {
        REQUIRE(kin.toSwerveCommand(cmd(1.0f, 2.0f), 0.02f, wheels));
        angle = wheels[0].steering_angle_rad;
        REQUIRE(std::fabs(angle - prev_angle) <= max_delta + 1e-5f);
        prev_angle = angle;
    }
    REQUIRE(angle == Approx(target).margin(1e-3f));
}

TEST_CASE_METHOD(TestFixture, "Tiny pure rotation hits deadband (documented gap)", "[kinematics][dynamics][gap]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    bool active = kin.toSwerveCommand(cmd(0.0f, 0.02f), 1.0f, wheels);
    REQUIRE(active);
    REQUIRE(wheels[0].drive_velocity_mps == Approx(0.0f));
    REQUIRE(wheels[0].steering_angle_rad == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Small vx and omega below epsilon is idle", "[kinematics][dynamics]") {
    SwerveKinematics kin{fw_geom};
    SwerveCommand wheels[2];
    REQUIRE_FALSE(kin.toSwerveCommand(cmd(0.005f, 0.005f), 1.0f, wheels));
    REQUIRE(wheels[0].drive_velocity_mps == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Inverse: straight wheels give forward vx only", "[kinematics][inverse]") {
    SwerveKinematics kin{fw_geom};
    float angles[2] = {0.0f, 0.0f};
    float speeds[2] = {1.0f, 1.0f};
    BodyVelocity v = kin.fromWheelFeedback(angles, speeds);
    REQUIRE(v.vx_mps == Approx(1.0f));
    REQUIRE(v.omega_radps == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Inverse: opposite steering recovers omega from sin formula", "[kinematics][inverse]") {
    SwerveKinematics kin{fw_geom};
    float angles[2] = {0.3f, -0.3f};
    float speeds[2] = {1.0f, 1.0f};
    BodyVelocity v = kin.fromWheelFeedback(angles, speeds);
    REQUIRE(v.vx_mps == Approx(std::cos(0.3f)));
    REQUIRE(v.omega_radps == Approx(std::sin(0.3f) / kHalfL));
}

TEST_CASE_METHOD(TestFixture, "Inverse: identical wheel angles yield zero omega (firmware pattern, gap)", "[kinematics][inverse][gap]") {
    SwerveKinematics kin{fw_geom};
    float angles[2] = {0.3f, 0.3f};
    float speeds[2] = {1.0f, 1.0f};
    BodyVelocity v = kin.fromWheelFeedback(angles, speeds);
    REQUIRE(v.vx_mps == Approx(std::cos(0.3f)));
    REQUIRE(v.omega_radps == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Inverse: zero feedback gives zero body velocity", "[kinematics][inverse]") {
    SwerveKinematics kin{fw_geom};
    float angles[2] = {0.0f, 0.0f};
    float speeds[2] = {0.0f, 0.0f};
    BodyVelocity v = kin.fromWheelFeedback(angles, speeds);
    REQUIRE(v.vx_mps == Approx(0.0f));
    REQUIRE(v.omega_radps == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Round-trip: toSwerveCommand then fromWheelFeedback recovers input", "[kinematics][roundtrip]") {
    const float sweep[][2] = {
        {0.3f, 0.0f},
        {0.5f, 0.2f},
        {1.0f, 0.5f},
        {0.8f, -0.3f},
        {1.0f, 2.0f},
    };
    for (const auto& p : sweep) {
        SwerveKinematics kin{fw_geom};
        SwerveCommand wheels[2];
        REQUIRE(kin.toSwerveCommand(cmd(p[0], p[1]), 1.0f, wheels));
        float angles[2] = {wheels[0].steering_angle_rad, wheels[1].steering_angle_rad};
        float speeds[2] = {wheels[0].drive_velocity_mps, wheels[1].drive_velocity_mps};
        BodyVelocity rec = kin.fromWheelFeedback(angles, speeds);
        INFO("point vx=" << p[0] << " omega=" << p[1]);
        REQUIRE(rec.vx_mps == Approx(p[0]).margin(1e-3f));
        REQUIRE(rec.omega_radps == Approx(p[1]).margin(1e-3f));
    }
}
