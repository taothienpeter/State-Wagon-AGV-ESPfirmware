#include "catch.hpp"
#include "trajectory_gen.h"
#include <cmath>

namespace {
constexpr float kPi = 3.14159265358979f;
constexpr float kPiOver3 = 1.04719755f;
}

struct TestFixture {
    MotionProfile profile{0.5f, 0.8f, 1.0f};
    TrajectoryGen gen{profile};

    static Waypoint wp(float x, float y, float max_speed = 1.0f, float tolerance = 0.05f) {
        return Waypoint{x, y, max_speed, tolerance};
    }

    TrajectoryGen& loadSingle(float x, float y, float max_speed = 1.0f) {
        gen.loadWaypoints({wp(x, y, max_speed)});
        return gen;
    }

    struct Pose {
        float v[3];
        operator const float*() const { return v; }
    };

    static Pose pose(float x, float y, float theta) {
        return Pose{{x, y, theta}};
    }

    static void integrate(float p[3], const BodyVelocity& v, float dt) {
        p[0] += v.vx_mps * std::cos(p[2]) * dt;
        p[1] += v.vx_mps * std::sin(p[2]) * dt;
        p[2] += v.omega_radps * dt;
    }
};

TEST_CASE_METHOD(TestFixture, "Fresh generator is idle and inert", "[trajectory][state]") {
    REQUIRE(gen.status() == TrajStatus::IDLE);
    REQUIRE(gen.activeIndex() == -1);
    REQUIRE(gen.currentSpeedMps() == Approx(0.0f));

    auto out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.vx_mps == Approx(0.0f));
    REQUIRE(out.omega_radps == Approx(0.0f));
    REQUIRE(gen.status() == TrajStatus::IDLE);
}

TEST_CASE_METHOD(TestFixture, "loadWaypoints drives state", "[trajectory][state]") {
    gen.loadWaypoints({});
    REQUIRE(gen.status() == TrajStatus::IDLE);
    REQUIRE(gen.activeIndex() == -1);
    auto out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.vx_mps == Approx(0.0f));
    REQUIRE(out.omega_radps == Approx(0.0f));

    loadSingle(10.0f, 0.0f);
    REQUIRE(gen.status() == TrajStatus::ACTIVE);
    REQUIRE(gen.activeIndex() == 0);

    gen.loadWaypoints({wp(5.0f, 0.0f), wp(6.0f, 0.0f)});
    REQUIRE(gen.status() == TrajStatus::ACTIVE);
    REQUIRE(gen.activeIndex() == 0);
    REQUIRE(gen.currentSpeedMps() == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Lifecycle: reset, pause, resume", "[trajectory][state]") {
    gen.reset();
    REQUIRE(gen.status() == TrajStatus::IDLE);
    REQUIRE(gen.activeIndex() == -1);
    REQUIRE(gen.currentSpeedMps() == Approx(0.0f));

    gen.resume();
    REQUIRE(gen.status() == TrajStatus::IDLE);

    loadSingle(10.0f, 0.0f);
    gen.pause();
    REQUIRE(gen.status() == TrajStatus::PAUSED);
    auto out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.vx_mps == Approx(0.0f));
    REQUIRE(out.omega_radps == Approx(0.0f));
    REQUIRE(gen.currentSpeedMps() == Approx(0.0f));

    gen.resume();
    REQUIRE(gen.status() == TrajStatus::ACTIVE);
}

TEST_CASE_METHOD(TestFixture, "Emergency stop is sticky until cleared", "[trajectory][state]") {
    loadSingle(10.0f, 0.0f);
    gen.emergencyStop();
    REQUIRE(gen.status() == TrajStatus::ESTOP);
    REQUIRE(gen.currentSpeedMps() == Approx(0.0f));

    for (int i = 0; i < 5; ++i) {
        auto out = gen.tick(pose(0, 0, 0), 0.02f);
        REQUIRE(out.vx_mps == Approx(0.0f));
        REQUIRE(out.omega_radps == Approx(0.0f));
        REQUIRE(gen.status() == TrajStatus::ESTOP);
    }

    gen.clearEstop();
    REQUIRE(gen.status() == TrajStatus::ACTIVE);

    gen.reset();
    gen.emergencyStop();
    gen.clearEstop();
    REQUIRE(gen.status() == TrajStatus::IDLE);
}

TEST_CASE_METHOD(TestFixture, "Acceleration ramps v = a*dt per tick up to max", "[trajectory][profile]") {
    loadSingle(10.0f, 0.0f);

    auto out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.vx_mps == Approx(0.5f * 0.02f));

    float expected = 0.5f * 0.02f;
    for (int i = 0; i < 250; ++i) {
        out = gen.tick(pose(0, 0, 0), 0.02f);
        expected = std::min(1.0f, expected + 0.5f * 0.02f);
        REQUIRE(out.vx_mps == Approx(expected));
        REQUIRE(out.vx_mps <= 1.0f);
    }
    REQUIRE(out.vx_mps == Approx(1.0f));
}

TEST_CASE_METHOD(TestFixture, "Cruise holds max speed while distance exceeds stopping distance", "[trajectory][profile]") {
    loadSingle(10.0f, 0.0f);
    float p[3] = {0, 0, 0};
    for (int i = 0; i < 120; ++i) {
        auto out = gen.tick(p, 0.02f);
        integrate(p, out, 0.02f);
    }
    float dist = std::sqrt((10.0f - p[0]) * (10.0f - p[0]) + p[1] * p[1]);
    REQUIRE(dist > (1.0f * 1.0f) / (2.0f * 0.8f));
    auto out = gen.tick(p, 0.02f);
    REQUIRE(out.vx_mps == Approx(1.0f));
}

TEST_CASE_METHOD(TestFixture, "dt=0 is a no-op and never yields NaN", "[trajectory][profile][edge]") {
    loadSingle(10.0f, 0.0f);
    auto out = gen.tick(pose(0, 0, 0), 0.0f);
    REQUIRE(out.vx_mps == Approx(0.0f));
    REQUIRE(out.vx_mps >= 0.0f);
    REQUIRE(!std::isnan(out.vx_mps));
    REQUIRE(!std::isnan(out.omega_radps));
}

TEST_CASE_METHOD(TestFixture, "Commanded speed is never negative", "[trajectory][profile][edge]") {
    loadSingle(10.0f, 0.0f);
    auto out = gen.tick(pose(0, 0, 0), -0.02f);
    REQUIRE(!std::isnan(out.vx_mps));
    REQUIRE(out.vx_mps >= 0.0f);
}

TEST_CASE_METHOD(TestFixture, "Deceleration follows v = sqrt(2*decel*dist) and is monotonic", "[trajectory][profile]") {
    loadSingle(5.0f, 0.0f);

    float p[3] = {0, 0, 0};
    float prev = 0.0f;
    bool decel_seen = false;
    bool monotonic = true;

    for (int i = 0; i < 5000 && gen.status() != TrajStatus::COMPLETE; ++i) {
        float dist = std::sqrt((5.0f - p[0]) * (5.0f - p[0]) + p[1] * p[1]);
        auto out = gen.tick(p, 0.02f);
        float v = out.vx_mps;
        if (gen.status() == TrajStatus::COMPLETE) break;

        REQUIRE(v >= 0.0f);
        if (decel_seen) {
            if (v > prev + 1e-4f) monotonic = false;
            REQUIRE(v == Approx(std::sqrt(2.0f * 0.8f * dist)));
        } else if (v < prev) {
            decel_seen = true;
            REQUIRE(dist <= (prev * prev) / (2.0f * 0.8f) + 0.001f);
            REQUIRE(v == Approx(std::sqrt(2.0f * 0.8f * dist)));
        }
        prev = v;
        integrate(p, out, 0.02f);
    }

    REQUIRE(decel_seen);
    REQUIRE(monotonic);
    REQUIRE(gen.status() == TrajStatus::COMPLETE);
    REQUIRE(gen.currentSpeedMps() == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Heading: dead ahead yields zero omega", "[trajectory][heading]") {
    loadSingle(10.0f, 0.0f);
    auto out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.omega_radps == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Heading: omega equals gain times heading error when unclamped", "[trajectory][heading]") {
    loadSingle(10.0f, 10.0f);
    auto out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.omega_radps == Approx(kPi / 4.0f));
}

TEST_CASE_METHOD(TestFixture, "Heading: error clamped to +/- pi/3", "[trajectory][heading]") {
    loadSingle(10.0f, 0.0f);
    auto out = gen.tick(pose(0, 0, 2.5f), 0.02f);
    REQUIRE(out.omega_radps == Approx(-kPiOver3));

    loadSingle(10.0f, 0.0f);
    out = gen.tick(pose(0, 0, -2.5f), 0.02f);
    REQUIRE(out.omega_radps == Approx(kPiOver3));
}

TEST_CASE_METHOD(TestFixture, "Heading: angle wrap maps error into (-pi, pi]", "[trajectory][heading]") {
    loadSingle(-1.0f, -0.1425f);
    auto out = gen.tick(pose(0, 0, 3.0f), 0.02f);
    REQUIRE(out.omega_radps == Approx(0.2832f).epsilon(0.001f));
}

TEST_CASE_METHOD(TestFixture, "Heading: omega magnitude never exceeds pi/3", "[trajectory][heading]") {
    loadSingle(10.0f, 0.0f);
    for (int i = -100; i <= 100; ++i) {
        float theta = i * (2.0f * kPi) / 100.0f;
        auto out = gen.tick(pose(0, 0, theta), 0.02f);
        REQUIRE(std::fabs(out.omega_radps) <= kPiOver3 + 1e-5f);
    }

    loadSingle(10.0f, 0.0f);
    auto out = gen.tick(pose(0, 0, 0.5f), 0.02f);
    REQUIRE(out.omega_radps == Approx(-0.5f));

    loadSingle(10.0f, 0.0f);
    out = gen.tick(pose(0, 0, -0.5f), 0.02f);
    REQUIRE(out.omega_radps == Approx(0.5f));
}

TEST_CASE_METHOD(TestFixture, "Arrival advances to next waypoint, completion zeroes output", "[trajectory][arrival]") {
    gen.loadWaypoints({wp(1.0f, 0.0f), wp(2.0f, 0.0f)});
    float p[3] = {0, 0, 0};
    for (int i = 0; i < 5000 && gen.activeIndex() == 0; ++i) {
        auto out = gen.tick(p, 0.02f);
        integrate(p, out, 0.02f);
    }
    REQUIRE(gen.activeIndex() == 1);
    REQUIRE(gen.status() == TrajStatus::ACTIVE);

    for (int i = 0; i < 5000 && gen.status() != TrajStatus::COMPLETE; ++i) {
        auto out = gen.tick(p, 0.02f);
        integrate(p, out, 0.02f);
    }
    REQUIRE(gen.status() == TrajStatus::COMPLETE);
    auto out = gen.tick(p, 0.02f);
    REQUIRE(out.vx_mps == Approx(0.0f));
    REQUIRE(out.omega_radps == Approx(0.0f));
    REQUIRE(gen.currentSpeedMps() == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Collocated waypoints clear in one tick with a single speed update", "[trajectory][arrival]") {
    gen.loadWaypoints({wp(0.02f, 0.0f), wp(0.04f, 0.0f), wp(10.0f, 10.0f)});
    auto out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(gen.activeIndex() == 2);
    REQUIRE(gen.status() == TrajStatus::ACTIVE);
    REQUIRE(out.vx_mps == Approx(0.5f * 0.02f));
    REQUIRE(out.omega_radps == Approx(kPi / 4.0f));

    auto next = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(next.vx_mps == Approx(2.0f * 0.5f * 0.02f));

    gen.loadWaypoints({wp(0.02f, 0.0f), wp(0.04f, 0.0f)});
    out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(gen.status() == TrajStatus::COMPLETE);
    REQUIRE(gen.activeIndex() == 2);
    REQUIRE(out.vx_mps == Approx(0.0f));
    REQUIRE(out.omega_radps == Approx(0.0f));
}

TEST_CASE_METHOD(TestFixture, "Speed obeys per-waypoint max speed and stays non-negative", "[trajectory][profile][arrival]") {
    gen.loadWaypoints({wp(0.5f, 0.0f, 1.0f), wp(1.0f, 0.0f, 0.5f), wp(3.0f, 0.0f, 1.0f)});
    float p[3] = {0, 0, 0};
    for (int i = 0; i < 5000 && gen.status() != TrajStatus::COMPLETE; ++i) {
        auto out = gen.tick(p, 0.02f);
        REQUIRE(out.vx_mps >= 0.0f);
        REQUIRE(out.vx_mps <= 1.0f + 1e-5f);
        if (gen.activeIndex() == 1)
            REQUIRE(out.vx_mps <= 0.5f + 1e-5f);
        integrate(p, out, 0.02f);
    }
    REQUIRE(gen.status() == TrajStatus::COMPLETE);
}

TEST_CASE_METHOD(TestFixture, "Output is zero in every non-active state", "[trajectory][output]") {
    loadSingle(10.0f, 0.0f);
    auto out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.vx_mps == Approx(gen.currentSpeedMps()));
    REQUIRE(out.vx_mps == Approx(0.5f * 0.02f));

    gen.pause();
    out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.vx_mps == Approx(0.0f));
    REQUIRE(out.omega_radps == Approx(0.0f));

    gen.emergencyStop();
    out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.vx_mps == Approx(0.0f));
    REQUIRE(out.omega_radps == Approx(0.0f));

    gen.reset();
    out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.vx_mps == Approx(0.0f));
    REQUIRE(out.omega_radps == Approx(0.0f));

    gen.loadWaypoints({});
    out = gen.tick(pose(0, 0, 0), 0.02f);
    REQUIRE(out.vx_mps == Approx(0.0f));
    REQUIRE(out.omega_radps == Approx(0.0f));
}
