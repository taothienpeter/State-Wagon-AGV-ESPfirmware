# AGV ESP32 Firmware v2

Three-tier swerve-drive AGV controller: Fleet Server (TCP/JSON) — ESP32 (trajectory, kinematics, sensor fusion) — STM32 (1kHz motor control).

## System Architecture

### Three-Tier Overview

```
┌──────────────────┐
│   Fleet Server   │  Hop1: TCP + 4-byte length-prefixed JSON
├──────────────────┤
│     ESP32        │  Hop2: Binary UART 921600 baud
├──────────────────┤
│     STM32        │  1kHz motor current loop
└──────────────────┘
```

- **Fleet Server**: Issues orders (waypoint lists) and receives state updates. No real-time control.
- **ESP32**: Trajectory generation at 50Hz, swerve kinematics, 6-state ES-EKF sensor fusion (odometry + IMU + UWB). Runs FreeRTOS on dual-core ESP32.
- **STM32**: 1kHz motor current/velocity loop, steering servo control, safety state machine. Receives motion commands from ESP32, sends back feedback (velocity, position, faults).

### Hop1 Protocol (ESP32 ↔ Server)

TCP connection with 4-byte big-endian length prefix followed by JSON. Envelope format:

```json
{
  "type": "connection|order|instantAction|state",
  "vehicleId": "agv-test-01",
  "payload": { ... }
}
```

**Message types:**
- `connection` — handshake on connect: `{"connectionState":"ONLINE"}`
- `order` — inbound: `{"orderId":"ord-0001","waypoints":[{"x":0,"y":0,"max_speed_mps":1.0}]}`
- `instantAction` — inbound: `{"actionType":"emergencyStop|startPause|stopPause|clearFault"}`
- `state` — outbound at 8Hz: position, velocity, battery, IMU heading, UWB position, drive controller status

### Hop2 Protocol (ESP32 ↔ STM32)

Binary UART at 921600 baud. Frame format:

```
[STX:1][VERSION:1][MSG_ID:1][SEQ:1][LEN:1][PAYLOAD:0-64][CRC16:2]
```

**Message IDs:**
| ID | Name | Direction | Payload |
|----|------|-----------|---------|
| 0x01 | MOTION_CMD | ESP32→STM32 | 18 bytes (mode, desired_speed, steering_angle, steering_velocity, timestamp, flags) |
| 0x02 | MOTION_FB | STM32→ESP32 | 25 bytes (drive velocity, steering actual, steering velocity, motor current, odrive Vbus, safety state, fault code, odrive errors, stepper homed) |
| 0x03 | SAFETY_EVENT | Bidirectional | Source byte |
| 0x04 | CONFIG_SET | ESP32→STM32 | 16 bytes (4 floats: wheel_radius, wheelbase, max_drive_velocity, max_steering_angle) |

CRC16-CCITT with 0x8408 polynomial (reflected form, equivalent to standard 0x1021).

## Component Map

```
main/
  main.cpp          — Task creation, callbacks, integration wiring
  config.h          — All compile-time configuration constants
  CMakeLists.txt

components/
  comm/
    hop1_client.*   — Hop1 TCP/JSON protocol client
  stm_link/
    protocol/
      stm_link.h    — Binary protocol structs, message IDs, CRC
    stm_uart_link.* — UART transport layer
  kinematics/
    swerve_kinematics.h  — Bicycle-model swerve kinematics (header-only)
  trajectory/
    trajectory_gen.*     — Trapezoidal speed profile trajectory generator
  imu/
    bno055_driver.*      — BNO055 IMU driver (I2C, NDOF fusion)
  uwb/
    dwm1000_driver.*     — DWM1000 UWB driver (SPI, 2D trilateration)
  es_ekf/
    es_ekf.hpp            — Error-State Extended Kalman Filter (6-state)
```

## FreeRTOS Task Layout

| Task | Core | Priority | Rate | Stack |
|------|------|----------|------|-------|
| planner | 1 | 10 | 50 Hz | 4096 |
| imu_sensor | 0 | 8 | 100 Hz | 4096 |
| uwb_ranging | 0 | 5 | 10 Hz | 4096 |
| state_pub | 0 | 5 | 8 Hz | 4096 |
| heartbeat | 0 | 3 | 1 Hz | 2048 |

Planner runs on Core 1; all sensor and communication tasks on Core 0.

## Data Flow (Planner Loop)

```
EKF pose [x, y, theta]
       │
       ▼
TrajectoryGen::tick(pose, dt_s)
       │
       ▼ BodyVelocity {vx_mps, omega_radps}
       │
SwerveKinematics::toSwerveCommand(body_v, dt_s, wheels[2])
       │
       ▼ motion_cmd_t {mode, speed, steering_angle, ...}
       │
       ▼
StmUartLink::sendMotionCmd(cmd)
```

## Sensor Fusion (ES-EKF)

6-state EKF: [x, y, theta, vx, vy, vtheta]

- **Predict**: Wheel odometry from STM32 feedback (drive velocity + steering angle)
- **Update (IMU)**: BNO055 Euler heading at 100Hz via I2C
- **Update (UWB)**: DWM1000 2D position at 10Hz via SPI, least-squares trilateration from 4 anchors
- **Joseph form**: Covariance update in Joseph form for numerical stability

Shared globals protected by spinlock (`ekf_mux`): pose, IMU heading, UWB position.

## Trajectory Generation

Trapezoidal speed profile per segment:
- Accelerate at `max_accel_mps2` to waypoint's `max_speed_mps`
- Cruise at target speed
- Decelerate at `max_decel_mps2` approaching waypoint
- Heading error clamped to ±π/3, multiplied by `steering_gain` → omega

States: IDLE → ACTIVE → PAUSED/COMPLETE/ESTOP

## Swerve Kinematics (Bicycle Model)

Two-wheel model (front + rear), no track width:
- `toSwerveCommand(vx, omega, dt_s)` → steering angle, drive velocity, steering rate
- Internal rate limiting via steering_rate_radps and dt_s

## Configuration

All constants in [`main/config.h`](main/config.h):
- Network: WiFi SSID/pass, server host/port, vehicle ID
- Timing: control rates, timeouts
- Pin assignments: UART, I2C, SPI
- Vehicle geometry: wheelbase, wheel radius, velocity/steering limits
- Motion profile: accel/decel rates, steering gain, arrival tolerance
- EKF: process noise, initial covariance

## Migration from VDA5050 (v1 → v2)

The original firmware used VDA5050 JSON protocols (node/edge graphs, StateReport, InstantActions) with a more complex STM32 protocol (6 message IDs including heartbeat and ACK/NACK). v2 replaces this with:

- **Flat waypoint lists** instead of VDA5050 node/edge graphs — `[{x, y, max_speed_mps}]`
- **Envelope format** `{type, vehicleId, payload}` instead of raw VDA5050 messages
- **Typed callbacks** `onOrder(order_id, waypoints)` / `onInstantAction(action_type)` instead of raw JSON parsing at the integration layer
- **TrajectoryGen::tick() → BodyVelocity** pipeline instead of direct SwerveCommand output
- **Simplified STM32 protocol**: 4 message IDs, no ACK/NACK, no heartbeat
- **config_set_t** reduced from 37 bytes (with PID gains) to 16 bytes (4 geometry floats only)
- **VehicleGeometry** simplified: no track_width (bicycle model), no mode/flags in SwerveCommand
- **Entire `components/order/`** component removed (VDA5050 parser no longer needed)

### Key Design Decisions

1. **publishState(const char\* json)** kept as string-passing rather than switching to `nlohmann::json` — the state builder in main.cpp already serializes to a string with ArduinoJson, so adding a second JSON library would add complexity for no benefit.

2. **ArduinoJson 6.21.6** used throughout — no migration to nlohmann/json. The ARCHITECTUREv2.md document uses `nlohmann::json` as pseudocode; the actual implementation uses the existing ArduinoJson dependency.

3. **Joseph form covariance update** in ES-EKF preserved from v1 — numerically stable, well-tested.

4. **Spinlock-protected shared globals** rather than message queues for sensor data — lower latency on the planner's critical path, adequate for the data rates involved (100Hz IMU, 10Hz UWB).

## Build

Requires ESP-IDF 6.0.0.

```bash
idf.py fullclean
idf.py build
idf.py -p PORT flash monitor
```

A devcontainer configuration is provided at `.devcontainer/devcontainer.json` with a ready-to-use ESP-IDF environment.

## Tests (Desktop)

The architecture specifies test files for component-level verification:
- `tests/test_stm_link.c` — Protocol codec (CRC16, frame encode/decode)
- `tests/test_swerve_kinematics.cpp` — Kinematics math (bicycle model, rate limiting)
- `tests/test_trajectory_gen.cpp` — Trajectory generator (trapezoidal profiles, arrival detection)
- `tests/test_integration.cpp` — Full pipeline (hop1 → trajectory → kinematics → stm_link)

These are built and run natively (not on ESP32 hardware). See `tests/` for build instructions.
