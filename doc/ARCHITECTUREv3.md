# AGV Swerve Drive — System Architecture & Protocol

**Status:** Reflects actual ESP32 firmware implementation as of the current commit.
This document defines the architecture across all three tiers (server, ESP32, STM32).
Struct sizes, wire formats, message types, component interfaces, and configuration
are specified as they exist in the codebase.

**Toolchains:**
- **ESP32**: ESP-IDF 6.0.0 (FreeRTOS, `idf.py build/flash/monitor`, C/C++ components)
- **STM32F103**: STM32CubeIDE (STM32CubeMX `.ioc` pin config + HAL, bare-metal super-loop)
- **Server**: Python 3 (TCP/JSON, single process)

---

## 1. OVERVIEW & THREE-TIER ARCHITECTURE

```
┌─────────────────────────────────────────────────────┐
│ TIER 1: Fleet Server (hop1_server.py)              │
│  - TCP listening on port 7800                       │
│  - One process, in-memory vehicle registry          │
│  - Issues orders (flat waypoint lists),             │
│    receives state updates at ~8 Hz                  │
└────────────────┬────────────────────────────────────┘
                 │ Hop 1: TCP + 4-byte length-prefixed JSON
                 ↓
┌────────────────────────────────────────────────────────┐
│ TIER 2: ESP32 — ESP-IDF                               │
│  - Receives orders from server (flat waypoint list)   │
│  - Trajectory generation (trapezoidal profiles)       │
│  - Swerve kinematics (body velocity → wheel commands) │
│  - Sensor fusion (wheel odometry + BNO055 IMU heading │
│    → ES_EKF; UWB optional/disabled by default)        │
│  - Publishes state to server at 8 Hz                  │
│  - Sends MOTION_CMD to STM32 at 50 Hz                 │
└────────────────┬────────────────────────────────────┘
                 │ Hop 2: UART 921600 baud, binary framed
                 │ (stm_link protocol, CRC16-CCITT)
                 ↓
┌────────────────────────────────────────────────────────┐
│ TIER 3: STM32F103 — STM32CubeIDE                      │
│  - 50 Hz main loop (matches MOTION_CMD rate)          │
│  - Stepper in Velocity Mode                            │
│  - ODrive ASCII UART: position control input_mode 3   │
│  - 150 ms watchdog timeout (safety-critical)          │
│  - Safety event reporting                             │
└────────────────────────────────────────────────────────┘
```

### 1.1 Trajectory Generation Strategy

Orders are sent as **flat waypoint lists** — `[{x, y, max_speed_mps}]`. The ESP32
stores the entire list in a heap-allocated `std::vector<Waypoint>`.

**Per-tick computation (50 Hz):**
1. Compute actual dt from `xTaskGetTickCount()` difference (not hardcoded)
2. Trapezoidal speed profile: accel → cruise → decel based on stopping distance
3. Compute heading error → omega via `steering_gain` (capped to ±60°)
4. Run swerve kinematics → wheel velocities in mps
5. Integrate wheel rps × dt → accumulated turns for ODrive position control
6. Transmit `MOTION_CMD` (20 bytes) to STM32

**Arrival detection:** Waypoint reached when distance < `tolerance_m` (default 0.05 m).
Auto-advances to next waypoint recursively on same tick.

**Heading error clamping:** Limited to ±60° (PI_OVER_3) to prevent excessive rotation
rates when the vehicle orientation diverges from the target heading.

**Accumulated turns:** The planner integrates `wheel_rps × dt_s` over time into a
monotonically increasing `accumulated_turns`. This is sent to ODrive as a position
setpoint, and ODrive's `input_pos_filter` handles the smoothing internally.

---

## 2. WIRE PROTOCOLS

### 2.1 Hop 1: TCP + Length-Prefixed JSON (Server ↔ ESP32)

**Transport:**
- TCP port 7800 (configurable)
- Frame: `[4-byte big-endian length][UTF-8 JSON payload]`
- No TLS (add in production if facility network requires it)
- ESP32 sends state at 8 Hz; if 3 consecutive seconds with no state, server
  marks vehicle `OFFLINE`
- **Receive Buffer on ESP32:** 16 KB persistent buffer (handles max 200 waypoints ~11.8 KB).

**Message Envelope** (all Hop 1 messages):
```json
{
  "type": "order|state|connection|instantAction|ack",
  "vehicleId": "agv-07",
  "payload": { ... }
}
```

**JSON Library:** The ESP32 uses **ArduinoJson 6** (managed component:
`bblanchon/arduinojson@^6`). Callbacks pass JSON strings (`const char*`), not
`cJSON` pointers. The server-facing interface is `publishState(const char* json)`.

#### 2.1.1 `order` (Server → ESP32)
```json
{
  "type": "order",
  "vehicleId": "agv-07",
  "payload": {
    "orderId": "ord-2026-0001",
    "waypoints": [
      {"x": 0.0, "y": 0.0, "max_speed_mps": 0.8, "tolerance_m": 0.05},
      {"x": 2.5, "y": 0.0, "max_speed_mps": 0.8, "tolerance_m": 0.05},
      {"x": 5.0, "y": 1.5, "max_speed_mps": 0.6, "tolerance_m": 0.05}
    ]
  }
}
```
Optional field: `tolerance_m` (per-waypoint, defaults to 0.05 m).

**Only flat waypoints are supported.** If curved paths are needed, the server
pre-computes them into dense waypoints (up to 200 points).

#### 2.1.2 `ack` (ESP32 → Server)
Sent immediately upon receiving an `order` (required for confirmed dispatch when `order_ack_enabled=true`):
```json
{
  "type": "ack",
  "vehicleId": "agv-07",
  "payload": {
    "orderId": "ord-2026-0001",
    "status": "ACCEPTED",
    "reason": "Optional rejection reason if status is REJECTED"
  }
}
```

#### 2.1.3 `state` (ESP32 → Server, ~8 Hz)
```json
{
  "type": "state",
  "vehicleId": "agv-07",
  "payload": {
    "orderId": "ord-2026-0001",
    "orderState": "ACTIVE",
    "operatingMode": "AUTOMATIC",
    "safetyState": "NORMAL",
    "position": {"x": 1.23, "y": 0.45, "theta": 0.17},
    "velocity": {"vx": 0.62, "vy": 0.0, "omega": 0.01},
    "battery": {"percentage": 85.0, "voltage": 24.8, "charging": false},
    "imu": {
      "accel": [0.0, 0.0, 9.8],
      "gyro": [0.0, 0.0, 0.0],
      "heading_rad": 0.17,
      "calibration_status": 3
    },
    "uwb": {"anchors": 4, "quality": 0.0, "x": 1.22, "y": 0.44},
    "driveControllers": [
      {
        "id": "stm32-main",
        "linkOk": true,
        "faultCode": 0,
        "odriveVbus": 24.8,
        "odriveErrors": 0,
        "stepperHomed": 1
      }
    ],
    "errors": []
  }
}
```

**Field Rules:**
- `orderId`: Set to JSON `null` when vehicle is IDLE (no active/completed order).
- `orderState`: `"IDLE"` | `"ACTIVE"` | `"PAUSED"` | `"COMPLETED"` | `"ESTOP"`.
- `operatingMode`: `"MANUAL"` during ESTOP / Pause, `"AUTOMATIC"` normally. *(Never use `"EMERGENCY"`).*
- `safetyState`: `"NORMAL"` | `"WARN"` | `"SAFE_STOP"` | `"FAULT_LATCHED"`.

#### 2.1.4 `connection` (both directions, handshake)
Sent by ESP32 on TCP connect with capabilities:
```json
{
  "type": "connection",
  "vehicleId": "agv-07",
  "payload": {
    "connectionState": "ONLINE",
    "capabilities": {
      "max_speed_mps": 1.2
    }
  }
}
```

#### 2.1.5 `instantAction` (Server → ESP32, high priority)
```json
{
  "type": "instantAction",
  "vehicleId": "agv-07",
  "payload": {
    "actionType": "emergencyStop|startPause|stopPause|clearFault"
  }
}
```
Processed immediately — e-stop is never queued. On `emergencyStop`, a
`MODE_SAFE_STOP` command is sent directly to STM32.

---

### 2.2 Hop 2: Binary Framed UART (ESP32 ↔ STM32)

**Transport:**
- UART 921600 baud, 8N1, no hardware flow control
- Frame: `[STX:1][VERSION:1][MSG_ID:1][SEQ:1][LEN:1][PAYLOAD:0-32][CRC16:2]`
  - STX = 0xAA, VERSION = 0x01
  - CRC16-CCITT: polynomial 0x1021, reflected (`crc ^= 0x8408`), init 0xFFFF
  - CRC covers bytes `[1..LEN+4]` (everything after STX)
- Byte-level state machine receiver (resync on STX)
- Max payload: 64 bytes

**Message IDs:**
| ID | Name | Direction | Rate | Payload Size |
|----|------|-----------|------|--------------|
| 0x01 | `MOTION_CMD` | ESP32→STM32 | 50 Hz | 20 bytes |
| 0x02 | `MOTION_FB` | STM32→ESP32 | 50 Hz (reply to each CMD) | 29 bytes |
| 0x03 | `SAFETY_EVENT` | STM32→ESP32 | async, on change | 6 bytes |
| 0x04 | `CONFIG_SET` | ESP32→STM32 | once at boot | 16 bytes |

#### 2.2.1 `MOTION_CMD` (20 bytes, packed)

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;               // 4
    float    drive_pos_target_turns;     // 4, accumulated position in turns
    float    steering_angle_rad;         // 4, range: -π/2 to +π/2
    float    steering_velocity_radps;    // 4, rate limit hint for stepper
    uint8_t  mode;                       // 1
    uint8_t  flags;                      // 1
    uint8_t  _pad[2];                    // 2, reserved
} motion_cmd_t;  // 20 bytes
```

**Mode values:**
- `MODE_IDLE` = 0x00
- `MODE_POSITION` = 0x01
- `MODE_SAFE_STOP` = 0x02

**Flags:** bit0 = steering_enable, bit1 = drive_enable (both = 0x03 when ACTIVE, 0x00 otherwise).

**Drive position semantics:**
- ESP32 trajectory generator integrates velocity over time each tick:
  ```
  wheel_rps = speed_mps / (2π × wheel_radius_m)
  accumulated_turns += wheel_rps × dt_s
  ```
- `drive_pos_target_turns` is a monotonically increasing/decreasing accumulated
  wheel position.
- STM32 passes this directly to ODrive as `p <axis> <turns> 0 0`.
- On `SAFE_STOP`: STM32 sets both ODrive axes to IDLE and holds stepper position.

**Watchdog:** STM32 must receive at least one `MOTION_CMD` every 150 ms.

#### 2.2.2 `MOTION_FB` (29 bytes, packed)

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;                 // 4
    float    steering_angle_actual_rad;    // 4
    float    drive_pos_actual_turns;       // 4, from ODrive encoder
    float    drive_velocity_actual_mps;    // 4, from ODrive encoder
    float    drive_current_a;              // 4
    float    odrive_vbus_v;                // 4
    uint16_t odrive_error_flags;           // 2
    uint8_t  stepper_homed;                // 1
    uint8_t  safety_state;                 // 1
    uint8_t  fault_code;                   // 1
} motion_fb_t;  // 29 bytes
```

**Safety state values:**
- `SAFETY_NORMAL` = 0x00
- `SAFETY_WARN` = 0x01
- `SAFE_STOP` = 0x02
- `FAULT_LATCHED` = 0x03

#### 2.2.3 `SAFETY_EVENT` (6 bytes, async)

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  source;     // 0=E_STOP, 1=DRIVE_FAULT, 2=STEPPER_STALL,
                         // 3=HOMING_FAILED, 4=CMD_TIMEOUT
    uint8_t  new_state;  // safety_state_t value
} safety_event_t;  // 6 bytes
```

#### 2.2.4 `CONFIG_SET` (16 bytes, sent once at boot)

```c
typedef struct __attribute__((packed)) {
    float drive_wheel_radius_m;
    float wheelbase_m;
    float max_drive_velocity_mps;
    float max_steering_angle_rad;
} config_set_t;  // 16 bytes
```

PID gains, ODrive filter bandwidth, and stepper limits are **not** sent over the
wire — they live in STM32 firmware `config.h`. A corrupted packet corrupting
control-loop gains is unacceptable for safety-critical systems.

#### 2.2.5 Compile-Time Verification

```c
_Static_assert(sizeof(motion_cmd_t) == 20, "motion_cmd_t must be 20 bytes");
_Static_assert(sizeof(motion_fb_t) == 29, "motion_fb_t must be 29 bytes");
_Static_assert(sizeof(safety_event_t) == 6, "safety_event_t must be 6 bytes");
_Static_assert(sizeof(config_set_t) == 16, "config_set_t must be 16 bytes");
```

Defined in `components/stm_link/protocol/stm_link.h`. Additionally, `static_assert`
for `motion_cmd_t` and `motion_fb_t` is checked at boot in `main.cpp:app_main()`.

#### 2.2.6 Encode/Decode Helpers (Inline)

`stm_link.h` provides inline `stm_link_encode()` and `stm_link_decode()` functions.
The decoder is a frame-level parser that returns one of:
- `STM_DECODE_OK` — complete valid frame
- `STM_DECODE_NEED_MORE` — incomplete frame, more bytes expected
- `STM_DECODE_BAD_STX` / `STM_DECODE_CRC_ERR` / `STM_DECODE_TOO_SHORT` — errors

---

### 2.3 Hop 3: ODrive Native ASCII Protocol (STM32 ↔ ODrive)

**Transport:**
- UART 115200 baud
- Line-oriented ASCII: `COMMAND [ARGS]\n`

**Drive Control: Position Control with `input_pos_filter` (Input Mode 3)**

ODrive axes are configured at STM32 boot into closed-loop position control.
The input position filter acts as a second-order low-pass smoother,
providing built-in acceleration limiting and ripple-free motion.

**Boot initialization (per axis):**
```
w axis0.requested_state 1\n              ; IDLE
w axis0.controller.config.control_mode 3\n      ; POSITION_CONTROL
w axis0.controller.config.input_mode 3\n        ; POS_FILTER
w axis0.controller.config.input_filter_bandwidth 5.0\n
w axis0.requested_state 8\n              ; CLOSED_LOOP_CONTROL
```

**Continuous setpoint (every MOTION_CMD, 50 Hz):**
```
p 0 <drive_pos_target_turns> 0 0\n
p 1 <drive_pos_target_turns> 0 0\n
```

**Emergency stop:**
```
w axis0.requested_state 1\n
w axis1.requested_state 1\n
```

---

## 3. FIRMWARE STRUCTURE & COMPONENTS

### 3.1 Directory Layout (ESP32)

```
esp32_agv/
├── CMakeLists.txt
├── sdkconfig
├── managed_components/
│   └── bblanchon__arduinojson/     (ArduinoJson 6.x via ESP-IDF component manager)
├── main/
│   ├── CMakeLists.txt
│   ├── main.cpp                    # app_main(), FreeRTOS tasks, callbacks
│   ├── config.h                    # WiFi, pins, vehicle geometry, timing
│   └── idf_component.yml
└── components/
    ├── comm/                       # Hop 1 (TCP/JSON via ArduinoJson)
    │   ├── CMakeLists.txt
    │   ├── hop1_client.h
    │   └── hop1_client.cpp
    ├── stm_link/                   # Hop 2 (UART binary)
    │   ├── CMakeLists.txt
    │   ├── protocol/
    │   │   └── stm_link.h          # Shared struct definitions (MASTER COPY)
    │   ├── stm_uart_link.h
    │   └── stm_uart_link.cpp
    ├── kinematics/                 # Swerve kinematics (header-only)
    │   ├── CMakeLists.txt
    │   └── swerve_kinematics.h
    ├── trajectory/
    │   ├── CMakeLists.txt
    │   ├── trajectory_gen.h
    │   └── trajectory_gen.cpp
    ├── imu/
    │   ├── CMakeLists.txt
    │   ├── bno055_driver.h
    │   └── bno055_driver.cpp
    ├── uwb/
    │   ├── CMakeLists.txt
    │   ├── dwm1000_driver.h
    │   └── dwm1000_driver.cpp
    └── es_ekf/
        ├── CMakeLists.txt
        ├── es_ekf.cpp
        └── include/es_ekf.hpp
```

**`stm_link.h` at `components/stm_link/protocol/stm_link.h` is the single source
of truth** for all struct definitions. The STM32 firmware must maintain an
identical copy. A struct size mismatch causes frame CRC rejection silently.

---

## 4. COMPONENT SPECIFICATIONS

### 4.1 Hop 1 Client — `components/comm/hop1_client.h/.cpp`

**JSON library:** ArduinoJson 6 (managed component).

```cpp
class Hop1Client {
public:
    Hop1Client(const char* vehicle_id,
               const char* server_host, uint16_t server_port,
               float max_speed_mps = 1.2f);
    ~Hop1Client();

    void begin();

    typedef std::function<void(const char* order_id,
                               const char* waypoints_json)> OrderCallback;
    typedef std::function<void(const char* action_type)> ActionCallback;
    void onOrder(OrderCallback cb);
    void onInstantAction(ActionCallback cb);

    void sendAck(const char* order_id, const char* status, const char* reason = nullptr);
    void publishState(const char* state_payload_json);

    Hop1State linkState() const;
    bool isConnected() const;
};
```

**Behavior:**
- FreeRTOS task created on `begin()`: core 0, priority 6, stack 8192
- Persistent TCP client with auto-reconnect and exponential backoff (500 ms → 15 s)
- Socket timeout: 5 seconds (`SO_RCVTIMEO`, `SO_SNDTIMEO`)
- Sends connection handshake on connect with `capabilities.max_speed_mps`
- Receives 4-byte big-endian length-prefixed JSON frames into persistent 16 KB buffer (`recv_buf_`)
- Zero-malloc parsing: parses into member `StaticJsonDocument<16384> incoming_doc_` and serializes waypoints into `char wps_scratch_[16384]`
- On order received: extracts `orderId`, serializes `waypoints` array into scratch buffer, passes to callback
- On instantAction: extracts `actionType`, passes to callback
- `sendAck`: sends `{"type":"ack","vehicleId":"...","payload":{"orderId":"...","status":"ACCEPTED|REJECTED"}}`
- `sendEnvelope`: thread-safe (protected by `send_mux_`), sends via TCP with loop handling partial sends

**State machine:**
- `DISCONNECTED` → `CONNECTING` (backoff 500 ms, doubles to 15 s)
- `CONNECTING` → `CONNECTED` (on successful connect + handshake)
- `CONNECTED` → `DISCONNECTED` (on recv error, socket closed)

### 4.2 STM32 UART Link — `components/stm_link/stm_uart_link.h/.cpp`

```cpp
class StmUartLink {
public:
    StmUartLink();

    void begin(uart_port_t uart_num, int tx_pin, int rx_pin,
               uint32_t baud, int cmd_rate_hz);

    void sendMotionCmd(const motion_cmd_t& cmd);
    void sendConfig(const config_set_t& cfg);
    bool getLatestFeedback(motion_fb_t& out, uint32_t max_age_ms = 200);

    bool hasPendingSafetyEvent();
    bool popSafetyEvent(safety_event_t& out);

    bool linkOk() const;
    uint32_t framesReceivedOk() const;
    uint32_t framesCrcError() const;
};
```

**Behavior:**
- One FreeRTOS task: RX (core 0, priority 8, stack 2048)
- RX: byte-level state machine, resync on STX, validates CRC
- TX: non-blocking `uart_write_bytes()`, no separate TX task
- Thread-safe feedback via spinlock
- Safety events queued via dedicated FreeRTOS queue
- `linkOk()`: returns true if last received frame within 300 ms
- CRC errors tracked in `rx_crc_err_count_`

**RX state machine** (`WAIT_STX` → `WAIT_HEADER` → `WAIT_PAYLOAD_CRC`):
- Resyncs to STX (0xAA) on any sync loss
- Validates payload length ≤ 64 bytes
- Dispatches `MSG_MOTION_FB` to feedback buffer, `MSG_SAFETY_EVENT` to queue
- CRC check via `stm_link_crc16()`

### 4.3 Swerve Kinematics — `components/kinematics/swerve_kinematics.h`

Header-only. Bicycle model, 2-wheel double-direct swerve.

```cpp
struct SwerveCommand {
    float steering_angle_rad = 0.0f;
    float drive_velocity_mps = 0.0f;  // velocity in m/s (NOT turns)
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
    SwerveKinematics();
    explicit SwerveKinematics(const VehicleGeometry& geom);

    void setGeometry(const VehicleGeometry& geom);
    const VehicleGeometry& geometry() const;

    // Forward: body velocity → per-wheel commands
    // Returns false when both vx and omega are ~zero (idle)
    bool toSwerveCommand(const BodyVelocity& cmd, float dt_s,
                         SwerveCommand wheels[2]);

    // Inverse: wheel feedback → body velocity (for EKF odometry)
    BodyVelocity fromWheelFeedback(const float steering_angles[2],
                                   const float drive_speeds[2]) const;
};
```

**Key behaviors:**
- `toSwerveCommand` outputs `drive_velocity_mps` (speed), NOT accumulated turns.
  Turns are integrated by the planner in `main.cpp`.
- Steering rate limiting: prevents angle jumps beyond `max_steering_rate_radps × dt`.
- Deadband: speeds below 0.01 mps → output zero, hold previous steering angle.
- `fromWheelFeedback`: bicycle model inverse — averages vx and omega from both wheels.
- Zero-motion detection: returns `false` when both `vx` and `omega ≈ 0`.

### 4.4 Trajectory Generator — `components/trajectory/trajectory_gen.h/.cpp`

```cpp
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
    ~TrajectoryGen();

    void loadWaypoints(const std::vector<Waypoint>& wps);
    void reset();
    void pause();
    void resume();
    void emergencyStop();
    void clearEstop();

    // Main tick (50 Hz): trapezoidal profile → BodyVelocity
    BodyVelocity tick(const float pose[3], float dt_s);

    TrajStatus status() const;
    const char* orderStateString() const;
    float currentSpeedMps() const;
    int activeIndex() const;
};
```

**Behavior & Thread Safety:**
- **Thread Safety:** Protected by FreeRTOS **Recursive Mutex** (`mux_`) under `#if defined(ESP_PLATFORM)`. All public methods take and release the recursive lock safely, preventing data races between `hop1_task` and `planner` task.
- **`orderStateString()`:** Maps internal state to server strings (`"IDLE"`, `"ACTIVE"`, `"PAUSED"`, `"COMPLETED"`, `"ESTOP"`).
- **Flat waypoints only.**
- **Trapezoidal speed profile:**
  - Computes stopping distance: `d_stop = v² / (2 × max_decel)`
  - If `dist_remaining ≤ d_stop`: decelerate using `√(2 × max_decel × dist)`
  - If `v < max_speed`: accelerate at `max_accel`
  - Otherwise: cruise at `max_speed`
- **Heading control:** Pure P-controller via `steering_gain × heading_error`.
  Heading error clamped to ±60° (`PI_OVER_3`).
- **Arrival detection:** Distance to waypoint < `tolerance_m` → advance index.
  Recursively processes next waypoint on same tick if multiple reached (recursive mutex prevents self-deadlock).
- **ESTOP:** Returns zero velocity, status set to ESTOP.

### 4.5 BNO055 IMU Driver — `components/imu/bno055_driver.h/.cpp`

```cpp
class Bno055Driver {
public:
    Bno055Driver(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
                 uint32_t clk_hz = 400000, uint8_t addr = 0x28);

    bool begin();
    bool readHeadingRad(float& heading_rad);
    bool readEuler(float& heading_deg, float& roll_deg, float& pitch_deg);
    bool readAccel(float& ax, float& ay, float& az);
    bool readGyro(float& gx, float& gy, float& gz);
    void getCalStatus(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag);
};
```

**Uses ESP-IDF new I2C master driver API** (`i2c_master_bus_handle_t`,
`i2c_master_dev_handle_t`).

**Register map:**
- `0x00` CHIP_ID (expect 0xA0)
- `0x1A`/`0x1B` EUL_Heading (int16, 1/16°)
- `0x35` CALIB_STAT
- `0x3D` OPR_MODE (0x0C = NDOF fusion)

**NACK retry:** Not yet implemented. The BNO055's internal MCU can NACK I2C
transactions for up to 100 ms after a mode switch (e.g., to NDOF). ESP-IDF's
I2C driver does not retry NACKs by default.

### 4.6 DWM1000 UWB Driver — `components/uwb/dwm1000_driver.h/.cpp`

```cpp
class Dwm1000Driver {
public:
    Dwm1000Driver(spi_host_device_t host, int cs, int irq, int rst = -1);

    bool begin();
    bool doRanging(float ranges_m[MAX_ANCHORS], int& count);
    bool trilaterate(const float ranges_m[], int count, float& x, float& y);
    void setAnchors(const float anchors[][3], int count);
};
```

- **`setAnchors` signature:** Takes `const float anchors[][3]` (array of `[x, y, z]`).
- **`MAX_ANCHORS`:** 8
- **`doRanging`:** Blocking TWR cycle, returns ranges in meters.
- **`trilaterate`:** Least-squares solution, minimum 3 anchors.

### 4.7 Error-State EKF — `components/es_ekf/es_ekf.cpp`

```cpp
class ES_EKF {
public:
    float x_nom[6];     // State: [x, y, theta, vx, vy, vtheta]

    void init(float dt, float init_x, float init_y, float init_theta);
    void predict(float delta_x, float delta_y, float delta_theta);
    void updateIMU(float imu_theta);
    void updateUWB(float uwb_x, float uwb_y, float uwb_theta);
};
```

**Covariance parameters:**
- `Q` (process): diagonal `[0.01, 0.01, 0.004, 0.1, 0.1, 0.05]`
- `R_uwb` (measurement): `[0.2, 0.2, 0.04]` for (x, y, theta)
- `R_imu` (measurement): `0.02` for theta
- `P` (initial): diagonal `0.1`

**Predict step:** Updates velocity from encoder deltas, propagates position via Euler
integration, normalizes theta to [-π, π]. Uses linearized Jacobian with sparse
block structure for efficiency.

**Update steps:** Joseph form `(I - KH)P(I - KH)' + KRK'` for numerical stability.
Both `updateIMU` and `updateUWB` normalize theta after injection.

### 4.8 ESP32 Main Entry — `main/main.cpp`

**`app_main()` sequence:**
1. NVS init (erase if corrupt)
2. `static_assert(sizeof(motion_cmd_t)==20 && sizeof(motion_fb_t)==29)` — compile-time struct verification
3. WiFi init (connect to SSID, 15 s timeout, continues without network on timeout)
4. STM32 UART link begin (921600 baud)
5. Send `CONFIG_SET` once at boot
6. Init EKF (dt=0.02, pose=0,0,0), kinematics (from vehicle geometry)
7. Start Hop1Client (creates its own task on core 0, pri 6)
8. Create FreeRTOS tasks

**FreeRTOS Task Layout:**

| Task | Core | Priority | Rate | Stack | Notes |
|------|------|----------|------|-------|-------|
| planner | 1 | 10 | 50 Hz | 6144 | Float math + EKF + kinematics |
| imu_sensor | 0 | 8 | 100 Hz | 4096 | I2C reads via Bno055Driver |
| uwb_ranging | 0 | 5 | 10 Hz | 4096 | SPI ranging cycle |
| state_pub | 0 | 5 | 8 Hz | 4096 | JSON build + publish |
| heartbeat | 0 | 3 | 1 Hz | 2048 | Logging only |
| hop1_task | 0 | 6 | event-driven | 8192 | TCP recv/send, created by Hop1Client::begin |
| stm_rx | 0 | 8 | event-driven | 2048 | UART RX, created by StmUartLink::begin |

#### Planner Loop (50 Hz, core 1, priority 10)

The planner computes **actual dt** from tick timing — not a hardcoded constant.
FreeRTOS scheduler jitter of ±2 ms would cause ~10% speed error with fixed dt.

```cpp
// Compute actual dt
static TickType_t last_tick = 0;
TickType_t now = xTaskGetTickCount();
float dt_s = (last_tick == 0) ? 0.02f
           : (now - last_tick) * portTICK_PERIOD_MS / 1000.0f;
if (dt_s < 0.001f) dt_s = 0.02f;  // clamp minimum
if (dt_s > 0.05f) dt_s = 0.05f;   // clamp maximum (spurious wake)
last_tick = now;

// 1. Read STM32 feedback → bicycle-model odometry via fromWheelFeedback()
motion_fb_t fb;
if (g_stm_link.getLatestFeedback(fb, 300)) {
    float angles[2] = {fb.steering_angle_actual_rad, fb.steering_angle_actual_rad};
    float speeds[2] = {fb.drive_velocity_actual_mps, fb.drive_velocity_actual_mps};
    BodyVelocity odom = g_kinematics.fromWheelFeedback(angles, speeds);
    delta_x = odom.vx_mps * cosf(theta) * dt_s;
    delta_y = odom.vx_mps * sinf(theta) * dt_s;
    delta_theta = odom.omega_radps * dt_s;
}

// 2. EKF predict
g_ekf.predict(delta_x, delta_y, delta_theta);

// 3. EKF update from IMU heading (if new data)
// 4. EKF update from UWB position (if new data)
// 5. Publish EKF pose to shared globals

// 6. Trajectory → BodyVelocity
BodyVelocity body_v = g_trajectory.tick(pose, dt_s);

// 7. Kinematics → wheel velocities
SwerveCommand wheels[2];
bool has_cmd = g_kinematics.toSwerveCommand(body_v, dt_s, wheels);

// 8. Integrate wheel velocity to accumulated turns
if (has_cmd) {
    float wheel_rps = wheels[0].drive_velocity_mps
                    / (2π × wheel_radius_m);
    accumulated_turns += wheel_rps * dt_s;
}

// 9. Pack and send MOTION_CMD
motion_cmd_t cmd;
cmd.drive_pos_target_turns = accumulated_turns;
cmd.steering_angle_rad = wheels[0].steering_angle_rad;
cmd.mode = (status == ACTIVE) ? MODE_POSITION : MODE_IDLE;
cmd.flags = (status == ACTIVE) ? 0x03 : 0x00;
g_stm_link.sendMotionCmd(cmd);
```

**On ESTOP:** Zero velocity, `MODE_SAFE_STOP` sent immediately.

#### Callbacks

Both callback types are registered before `Hop1Client::begin()` and fire from the
hop1_task context:

- **`onOrderReceived(order_id, waypoints_json)`:** Deserializes the waypoints JSON
  array into `std::vector<Waypoint>`, stores `orderId`, calls `trajectory.loadWaypoints()`.
- **`onInstantAction(action_type)`:** Dispatch strings:
  - `"emergencyStop"` → `trajectory.emergencyStop()` + immediate `MODE_SAFE_STOP` to STM32
  - `"startPause"` → `trajectory.pause()`
  - `"stopPause"` → `trajectory.resume()`
  - `"clearFault"` → `trajectory.clearEstop()`

---

## 5. CONFIGURATION

### 5.1 ESP32 (`main/config.h`)

```c
/* ---- Identity & Network ---- */
#define AGV_DEFAULT_VEHICLE_ID      "agv-test-01"
#define AGV_DEFAULT_WIFI_SSID       "Wifi_Nha_Ban"
#define AGV_DEFAULT_WIFI_PASS       "Mat_Khau_Wifi"
#define AGV_DEFAULT_SERVER_HOST     "192.168.1.100"
#define AGV_DEFAULT_SERVER_PORT     7800

/* ---- Timing ---- */
#define AGV_HEARTBEAT_PERIOD_MS     1000
#define AGV_RECONNECT_BACKOFF_MIN_MS 500
#define AGV_RECONNECT_BACKOFF_MAX_MS 15000
#define AGV_STATE_PUBLISH_HZ        8
#define AGV_WIFI_CONNECT_TIMEOUT_MS 15000

/* ---- UART (to STM32) ---- */
#define STM_UART_NUM                UART_NUM_1
#define STM_TX_PIN                  GPIO_NUM_17
#define STM_RX_PIN                  GPIO_NUM_16
#define STM_UART_BAUD               921600
#define STM_CMD_RATE_HZ             50
#define STM_FB_TIMEOUT_MS           300

/* ---- IMU (BNO055 I2C) ---- */
#define IMU_I2C_NUM                 I2C_NUM_0
#define IMU_I2C_SDA_PIN             GPIO_NUM_21
#define IMU_I2C_SCL_PIN             GPIO_NUM_22
#define IMU_I2C_CLOCK_HZ            400000
#define IMU_I2C_ADDR                0x28
#define IMU_SAMPLE_RATE_HZ          100

/* ---- UWB (DWM1000 SPI) ---- */
#define UWB_SPI_HOST                SPI2_HOST
#define UWB_SPI_MOSI                GPIO_NUM_23
#define UWB_SPI_MISO                GPIO_NUM_19
#define UWB_SPI_CLK                 GPIO_NUM_18
#define UWB_SPI_CS                  GPIO_NUM_5
#define UWB_SPI_IRQ                 GPIO_NUM_4
#define UWB_SPI_CLOCK_HZ           20000000
#define UWB_RATE_HZ                 10
#define UWB_ANCHOR_COUNT            4

/* ---- Vehicle geometry ---- */
#define AGV_WHEELBASE_M             0.42f
#define AGV_WHEEL_RADIUS_M          0.075f
#define AGV_MAX_VELOCITY_MPS        1.2f
#define AGV_MAX_STEERING_ANGLE_RAD  1.57f
#define AGV_MAX_STEERING_RATE_RADPS 3.14f

/* ---- Motion profile ---- */
#define AGV_MAX_ACCEL_MPS2          0.5f
#define AGV_MAX_DECEL_MPS2          0.8f
#define AGV_ARRIVAL_TOLERANCE_M     0.05f
#define AGV_STEERING_GAIN           1.0f

/* ---- EKF defaults ---- */
#define EKF_DT_S                    0.02f
#define EKF_INIT_P                  0.1f
#define EKF_Q_POS                   0.01f
#define EKF_Q_HEADING               0.004f
#define EKF_Q_VEL                   0.1f
#define EKF_Q_VEL_HEADING           0.05f
```

### 5.2 STM32 (`Core/Inc/config.h`)

Pin assignments match the `.ioc` file. PID gains, accel/decel limits, and ODrive
filter bandwidth live here (compiled-in, never sent over UART).

---

## 6. BUILD & DEPLOY

**ESP32:**
```bash
cd esp32_agv
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

**STM32:** Import into STM32CubeIDE, Build (Ctrl+B), Flash (F11) via ST-Link.

**Server:**
```bash
python3 server/hop1_server.py
```

**Commissioning checklist:**
- [ ] Verify `static_assert` passes at compile time (struct size mismatch = compile error)
- [ ] ESP32 connects to WiFi, server receives `connection` handshake
- [ ] Send test order with waypoints → ESP32 generates MOTION_CMD at 50 Hz
- [ ] STM32 receives frames, replies with MOTION_FB
- [ ] ODrive receives `p <axis> <turns>` position commands
- [ ] State JSON published back to server at 8 Hz
- [ ] E-stop via instantAction → motors stop immediately
- [ ] Monitor planner task stack high watermark (should stay above 1024 bytes)
- [ ] Verify dt_s reported in logs stays within 18–22 ms at normal 50 Hz rate

---

## 7. KEY DESIGN DECISIONS & RATIONALE

### 7.1 Why Flat Waypoints (Not VDA5050 or Geometric Segments)

- **Simpler:** No graph traversal, no segment type dispatch. A waypoint is just
  `{x, y, max_speed_mps}`.
- **Smooth enough:** The heading-error P-controller naturally rounds corners.
  For curved paths, the server pre-computes dense waypoints.
- **One fewer failure mode:** No complex parser for segment types.

### 7.2 Why Position Control (Not Velocity)

ODrive's `input_pos_filter` (Input Mode 3) provides hardware-accelerated trajectory
smoothing at no firmware cost. The ESP32 integrates velocity into accumulated turns
and sends position setpoints. ODrive handles acceleration/deceleration internally.

### 7.3 Why ArduinoJson (Not cJSON)

ArduinoJson 6 is used as a managed ESP-IDF component. It provides a cleaner C++ API
and is well-tested in the ESP-IDF ecosystem. All JSON serialization/deserialization
is done via `DynamicJsonDocument` with `serializeJson`/`deserializeJson`.

### 7.4 Why PID Gains Never Go Over the Wire

A corrupted CONFIG_SET containing PID gains would cause uncontrolled oscillation
or instability. Geometry constants (wheel radius, wheelbase) are safe to send
because incorrect values cause gradual drift (corrected by EKF). Control loop
gains are tuned once and compiled into STM32 firmware.

### 7.5 Why Planner Uses Real dt (Not Fixed 0.02)

The planner runs at 50 Hz but `vTaskDelayUntil` is subject to FreeRTOS scheduler
jitter (±2 ms). Hardcoding `dt_s = 0.02f` causes accumulated trajectory error
proportional to jitter. Computing actual dt from `xTaskGetTickCount()` differences
eliminates this error source. Clamped to [1 ms, 50 ms] to handle spurious wakes.

### 7.6 Why Accumulated Turns (Not Raw Speed)

ODrive position control (`p <axis> <turns>`) expects a position setpoint, not a
velocity target. The ESP32 integrates `speed_mps → wheel_rps → turns` and sends
the accumulated turns value. ODrive's `input_pos_filter` provides the necessary
trajectory smoothing internally.

---

## 8. CURRENT STATUS & KNOWN LIMITATIONS

### 8.1 Implemented & Verified

- ✅ `stm_link.h` struct sizes: `motion_cmd_t` = 20 bytes, `motion_fb_t` = 29 bytes
- ✅ `_Static_assert` at compile time + `static_assert` at boot for struct sizes
- ✅ `MODE_POSITION` (0x01) with accumulated turns integration
- ✅ Actual dt computed from `xTaskGetTickCount()` difference
- ✅ `fromWheelFeedback()` used for EKF odometry (bicycle model)
- ✅ Planner task stack 6144 (core 1, pri 10)
- ✅ `STM_CMD_RATE_HZ` = 50 Hz
- ✅ Hop 1: TCP + ArduinoJson 6, order/state/connection/instantAction
- ✅ Hop 2: 4 message types via UART, CRC16-CCITT, byte-level state machine
- ✅ Trajectory generator: trapezoidal profile, heading clamp ±60°, arrival detection
- ✅ EKF: 6-state ES_EKF with predict, updateIMU, updateUWB
- ✅ Swerve kinematics: forward/inverse, rate limiting, deadband
- ✅ WiFi init with timeout, auto-reconnect in Hop1Client
- ✅ CONFIG_SET sent at boot
- ✅ Emergency stop via instantAction → MODE_SAFE_STOP

### 8.2 Placeholders (Not Yet Implemented)

| Item | Location | Notes |
|------|----------|-------|
| Battery percentage | `main.cpp:298` | Hardcoded to 85.0 — needs ADC-based monitoring |
| IMU calibration status | `main.cpp:303` | Hardcoded to 3 — need to read actual CALIB_STAT register |
| I2C NACK retry for BNO055 | `bno055_driver.cpp` | BNO055 can NACK during mode switch (CONFIG→NDOF), ESP-IDF I2C driver does not auto-retry |
| STM32 firmware | not in this repo | Stepper, ODrive ASCII, safety logic — reference architecture only |
| UWB hardware integration | `dwm1000_driver.cpp` | SPI and IRQ pin defined but not yet tested with physical DWM1000 modules |

### 8.3 Tests

No automated test suite is present in the repository. The reference architecture
specified desktop tests for CRC16, kinematics, trajectory, and integration, but
these have not been implemented yet.

### 8.4 Future Work

- [ ] Implement I2C NACK retry wrapper in BNO055 driver
- [ ] Read actual battery voltage via ADC, compute percentage
- [ ] Read actual BNO055 calibration status
- [ ] UWB hardware bring-up and anchor calibration
- [ ] Implement desktop test suite (CRC, kinematics, trajectory, integration)
- [ ] OTA firmware update (dual-app slots on ESP32)
- [ ] Planner stack high watermark monitoring during operation
- [ ] Multi-vehicle fleet management, Web UI dashboard

---

994: 5. Verify `sizeof(motion_cmd_t) == 20` and `sizeof(motion_fb_t) == 29` match
995:    on both ESP32 and STM32 compilers.
996: 
997: ---
998: 
999: ## 10. FULL SYSTEM INTEGRATION CHECKLIST & BENCH-TEST FLAGS
1000: 
1001: When moving from standalone bench testing to full vehicle integration (ESP32 + STM32 + ODrive + BNO055 + DWM1000), review and configure the following flags:
1002: 
1003: 1. **`FLAG-01`: STM32 UART Feedback Safety Guard (`main/main.cpp:307`)**
1004:    - *Bench-test mode (Current)*: `safetyState` defaults to `"NORMAL"` when `fb_valid == false` to allow standalone UI and trajectory testing without STM32 UART transceiver.
1005:    - *Full System Integration*: Restore `safety_state = fb_valid ? fb.safety_state : 2` (`SAFE_STOP`). Any loss of UART communications (>500ms) will immediately trigger safe stop.
1006: 
1007: 2. **`FLAG-02`: UWB 3-Source Sensor Fusion (`main/config.h:33`)**
1008:    - *Bench-test mode (Current)*: `#define AGV_ENABLE_UWB 0` (EKF runs on Wheel Odometry + BNO055 IMU Yaw).
1009:    - *Full System Integration*: Set `#define AGV_ENABLE_UWB 1`, set anchor count `#define UWB_ANCHOR_COUNT 4`, and measure real anchor positions `(x, y, z)` in `main/main.cpp:230-235`.
1010: 
1011: 3. **`FLAG-03`: Permanent Bug Fixes Retained**
1012:    - TCP Socket Keep-Alive (replaces `SO_RCVTIMEO` disconnect trap `BUG-16`).
1013:    - WiFi Modem Sleep Disabled (`WIFI_PS_NONE` in `main.cpp:512` - `BUG-17`).
1014:    - Zero-malloc JSON serialization across all cyclic tasks (`BUG-11`, `BUG-14`).
1015:    - Dedicated FreeRTOS Queue for STM32 safety events (`BUG-01`).
1016:    - FreeRTOS Recursive Mutex on `TrajectoryGen` (`BUG-10`).
1017: 
