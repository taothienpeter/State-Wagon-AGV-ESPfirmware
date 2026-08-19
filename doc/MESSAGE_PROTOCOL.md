# Message Protocol Specification (Current & Unified)

> **Cập nhật:** 2026-08-19  
> Tài liệu này mô tả toàn bộ giao thức trên đường truyền (wire format) qua 3 tầng giao tiếp:
> 1. **Hop 1**: Server (Python) $\Longleftrightarrow$ ESP32 Firmware (TCP/JSON, Length-prefixed)
> 2. **Hop 2**: ESP32 $\Longleftrightarrow$ STM32 Drive Controller (`stm_link`, Binary UART 921600 baud, CRC16)
> 3. **Hop 3**: STM32 $\Longleftrightarrow$ ODrive Actuators (ODrive ASCII UART)

```
[StageWagonServer] <==TCP/WiFi:7800==> [ESP32] <==UART:921600==> [STM32] <==UART==> [ODrive]
                       Hop 1                  Hop 2 (`stm_link`)           Hop 3
                  (JSON Length-prefixed)      (Binary CRC16-CCITT)      (ODrive ASCII)
```

---

## 1. Hop 1: Server ↔ ESP32 (`AGV_PROTO`, TCP over WiFi)

### 1.1 Framing
- **Framing**: `[4-byte big-endian length][UTF-8 JSON payload]`.
- **Cổng TCP**: `7800` (Persistent TCP connection do ESP32 kết nối tới Server).
- **Kích thước payload tối đa**: 1 MiB (Server), Buffer nhận trên ESP32: **16384 bytes** (đủ chứa order 200 waypoint bo góc ~11.8 KB).

### 1.2 Cấu trúc Envelope chung
Mọi message qua TCP đều tuân thủ envelope:
```json
{
  "type": "order | state | connection | instantAction | ack",
  "vehicleId": "agv-07",
  "payload": { ... }
}
```

---

### 1.3 Chi tiết các Message Hop 1

#### ① `connection` (ESP32 → Server)
Gửi ngay khi ESP32 kết nối TCP thành công để đăng ký trạng thái ONLINE và khai báo giới hạn vận tốc:
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

#### ② `order` (Server → ESP32)
Phát lệnh di chuyển theo danh sách waypoint phẳng (do MotionPlanner MPG sinh tối đa 200 điểm):
```json
{
  "type": "order",
  "vehicleId": "agv-07",
  "payload": {
    "orderId": "ord-1770000000-0001",
    "waypoints": [
      {"x": 0.0, "y": 0.0, "max_speed_mps": 0.8, "tolerance_m": 0.05},
      {"x": 2.5, "y": 0.0, "max_speed_mps": 0.8, "tolerance_m": 0.05},
      {"x": 5.0, "y": 1.5, "max_speed_mps": 0.6, "tolerance_m": 0.05}
    ]
  }
}
```

#### ③ `ack` (ESP32 → Server)
Phản hồi xác nhận ngay khi nhận `order` (phục vụ cơ chế Confirmed Dispatch khi bật `order_ack_enabled=true`):
```json
{
  "type": "ack",
  "vehicleId": "agv-07",
  "payload": {
    "orderId": "ord-1770000000-0001",
    "status": "ACCEPTED",
    "reason": "Optional rejection reason"
  }
}
```
* `status`: `"ACCEPTED"` | `"REJECTED"`.

#### ④ `instantAction` (Server → ESP32)
Lệnh điều khiển tức thời, độ ưu tiên cao, xử lý ngay lập tức:
```json
{
  "type": "instantAction",
  "vehicleId": "agv-07",
  "payload": {
    "actionType": "emergencyStop | startPause | stopPause | clearFault"
  }
}
```

#### ⑤ `state` (ESP32 → Server, định kỳ ~8 Hz)
Báo cáo toàn diện trạng thái định vị, động học, pin, cảm biến và actuator:
```json
{
  "type": "state",
  "vehicleId": "agv-07",
  "payload": {
    "orderId": "ord-1770000000-0001",
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
* **Quy tắc quan trọng**:
  * Khi xe IDLE không có order: `orderId` là JSON `null` (không gửi `""` để tránh sinh bản ghi rác trên server).
  * `orderState`: `"IDLE"` | `"ACTIVE"` | `"PAUSED"` | `"COMPLETED"` | `"ESTOP"`.
  * `operatingMode`: `"AUTOMATIC"` | `"MANUAL"` (khi ESTOP hoặc Pause). *Không dùng `"EMERGENCY"`.*
  * `safetyState`: `"NORMAL"` | `"WARN"` | `"SAFE_STOP"` | `"FAULT_LATCHED"`.

---

## 2. Hop 2: ESP32 ↔ STM32 (`stm_link`, Binary UART)

### 2.1 Cấu trúc Frame nhị phân
- **Baudrate**: `921600` baud, 8N1.
- **Định dạng Frame**:
```
[STX: 0xAA] [VERSION: 0x01] [MSG_ID: 1B] [SEQ: 1B] [LEN: 1B] [PAYLOAD: 0-64B] [CRC16: 2B]
```
- **CRC16-CCITT**: Khởi tạo `0xFFFF`, đa thức `0x1021`, tính từ `VERSION` đến hết `PAYLOAD`.

### 2.2 Danh mục Message IDs
| MSG_ID | Tên Message | Hướng | Tần suất | Payload Size |
|:---:|---|:---:|:---:|:---:|
| `0x01` | `MSG_MOTION_CMD` | ESP32 → STM32 | 50 Hz | 20 bytes |
| `0x02` | `MSG_MOTION_FB` | STM32 → ESP32 | 50 Hz | 29 bytes |
| `0x03` | `MSG_SAFETY_EVENT` | STM32 → ESP32 | Bất đồng bộ | 6 bytes |
| `0x04` | `MSG_CONFIG_SET` | ESP32 → STM32 | Khi khởi động | 16 bytes |

### 2.3 Chi tiết các Struct C/C++ (`stm_link.h`)

#### `motion_cmd_t` (20 bytes, packed)
```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;               // 4B, timestamp ESP32
    float    drive_pos_target_turns;     // 4B, vị trí tích lũy bánh ODrive (turns)
    float    steering_angle_rad;         // 4B, góc bẻ lái stepper (-π/2 đến +π/2)
    float    steering_velocity_radps;    // 4B, tốc độ bẻ lái stepper
    uint8_t  mode;                       // 1B, 0=IDLE, 1=POSITION, 2=SAFE_STOP
    uint8_t  flags;                      // 1B, bit0=steering_en, bit1=drive_en
    uint8_t  _pad[2];                    // 2B, reserved
} motion_cmd_t;
```

#### `motion_fb_t` (29 bytes, packed)
```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;                 // 4B, timestamp STM32
    float    steering_angle_actual_rad;    // 4B, góc lái thực tế
    float    drive_pos_actual_turns;       // 4B, vị trí thực tế ODrive (turns)
    float    drive_velocity_actual_mps;    // 4B, vận tốc bánh thực tế (m/s)
    float    drive_current_a;              // 4B, dòng điện động cơ (A)
    float    odrive_vbus_v;                // 4B, điện áp bus ODrive (V)
    uint16_t odrive_error_flags;           // 2B, cờ lỗi ODrive
    uint8_t  stepper_homed;                // 1B, cờ homing stepper (0/1)
    uint8_t  safety_state;                 // 1B, 0=NORMAL, 1=WARN, 2=SAFE_STOP, 3=FAULT_LATCHED
    uint8_t  fault_code;                   // 1B, mã lỗi cục bộ STM32
} motion_fb_t;
```

#### `safety_event_t` (6 bytes, packed)
```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  source;        // 0=E_STOP, 1=DRIVE_FAULT, 2=STEPPER_STALL, 3=HOMING_FAILED, 4=CMD_TIMEOUT
    uint8_t  new_state;     // Trạng thái an toàn mới
} safety_event_t;
```

#### `config_set_t` (16 bytes, packed)
```c
typedef struct __attribute__((packed)) {
    float drive_wheel_radius_m;
    float wheelbase_m;
    float max_drive_velocity_mps;
    float max_steering_angle_rad;
} config_set_t;
```

---

## 3. Hop 3: STM32 ↔ ODrive (ODrive Native ASCII UART)

- **Baudrate**: 921600 baud (hoặc 115200).
- **Lệnh điều khiển**:
  - `p 0 <pos_turns> 0 0` — Điều khiển vị trí bánh chủ động (`INPUT_MODE_POS_FILTER`).
  - `r vbus_voltage` — Đọc điện áp nguồn pin.
  - `r axis0.error` — Đọc cờ lỗi động cơ.
  - `r axis0.encoder.pos_estimate` — Đọc vị trí thực tế.
  - `r axis0.encoder.vel_estimate` — Đọc vận tốc thực tế.
