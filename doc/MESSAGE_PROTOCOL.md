# Message Protocol Specification

This document specifies every message on the wire, for every hop:

```
[RMS] <-internal-> [RCS] <==TCP/WiFi==> [ESP32] <==UART==> [STM32] <==UART==> [ODrive]
                          AGV_PROTO/1              stm_link            ODrive ASCII
                          (JSON, framed)           (binary, framed)    (documented,
                                                                          unmodified)
```

## 0. Design research: how real fleets structure this

Commercial AGV/AMR fleets standardized this exact Server↔Vehicle link as
**VDA 5050** (VDA + VDMA, first published 2019, current major version 2.x),
now widely implemented by AGV/AMR vendors and adopted by fleet-management
platforms specifically so that vehicles from different manufacturers can be
dispatched by one master control. It runs over **MQTT with JSON payloads**
and defines a small, fixed set of message/topic types:

- **`order`** — control→vehicle: a sequence of `nodes` (waypoints, each with
  an id, sequence id, position, and optional `actions`) and `edges`
  (connections between nodes, with max speed / trajectory hints)
- **`instantActions`** — control→vehicle: short, high-priority commands that
  don't wait for the current order (`cancelOrder`, `startPause`,
  `stopPause`, factsheet request, custom actions)
- **`state`** — vehicle→control: the full status report — position,
  velocity, battery, order/edge/node progress, active errors/warnings,
  operating mode, safety state
- **`connection`** — vehicle→control (via broker "last will"): online/
  offline/connection-broken, so control knows a vehicle silently died
- **`visualization`** — vehicle→control: a high-rate, "fire and forget"
  position stream for UI/monitoring, decoupled from `state` so UI refresh
  rate doesn't burden the authoritative state channel
- **`factsheet`** — vehicle→control: static vehicle capability description
  (dimensions, max speed, supported actions) exchanged once at
  registration

Every VDA 5050 message shares a common **header** — `headerId` (monotonic
counter), `timestamp`, `version`, `manufacturer`, `serialNumber` — regardless
of which of the 6 message types it is. That "one envelope, many typed
bodies" pattern is exactly what we replicate below across *all three hops*
of this system, adapted to each transport's constraints (JSON is fine at
the WiFi/TCP hop; a Cortex-M running a 1 kHz control loop needs a binary,
fixed-size, CRC'd frame instead of a JSON parser in the hot path).

For the STM32↔ODrive hop we do not invent anything: ODrive exposes a
documented, versioned **native UART ASCII protocol** (line-oriented,
`command *checksum ; comment\n`, e.g. `v 0 10.0 0.0` = "axis 0 velocity
setpoint 10.0 turns/s, 0.0 A torque feedforward", `r vbus_voltage` = "read
bus voltage") at 115200 baud by default, configurable up to 921600. We use
it unmodified as the actuator-facing protocol; the STM32 is the sole UART
master on that bus.

## 1. Common envelope pattern (applies at every hop)

| Field | Meaning | Present in |
|---|---|---|
| `type` / `msg_id` | discriminates payload schema | all |
| `seq` | monotonic counter, per-link | all |
| `timestamp` | producer-side timestamp (ms since boot or epoch) | all |
| `version` | protocol version byte/field | all |
| `payload` | type-specific body | all |
| checksum/CRC | integrity check | UART hops (mandatory), TCP hop (via TCP itself + optional payload hash for audit) |

The JSON hop additionally carries `robotId`/`vehicleId` since one server
process multiplexes many vehicles; the UART hops don't need an address
field (point-to-point wire).

---

## 2. Hop 1 — RCS ↔ ESP32 (`AGV_PROTO/1`, TCP over WiFi, JSON, length-prefixed)

### 2.1 Transport framing

Raw TCP is a byte stream, not a message stream, so every JSON message is
wrapped with a 4-byte big-endian length prefix:

```
+----------------+----------------------------+
| length (u32BE) | UTF-8 JSON payload (length) |
+----------------+----------------------------+
```

TCP port `7800` (configurable), one persistent connection per AGV,
ESP32-initiated (client), server listens. TLS is **recommended** for
production (`AGV_TLS=1`, mutual cert auth so a rogue device cannot spoof a
vehicle) — see `server/common/config.py` and `firmware/esp32/include/
config.h`; the default dev profile in this repo runs plaintext TCP for lab
bring-up, matching "currently on my laptop."

### 2.2 Common header (every JSON message)

```json
{
  "header": {
    "type": "order | instantAction | state | connection | telemetry | ack",
    "headerId": 1042,
    "timestamp": "2026-07-21T10:15:32.123Z",
    "version": "1.0.0",
    "vehicleId": "agv-07"
  },
  "payload": { "...": "type-specific, see below" }
}
```

### 2.3 `order` (RCS → ESP32)

```json
{
  "header": {"type": "order", "headerId": 1042, "timestamp": "...", "version": "1.0.0", "vehicleId": "agv-07"},
  "payload": {
    "orderId": "ord-2026-0007-014",
    "orderUpdateId": 0,
    "nodes": [
      {"nodeId": "N12", "sequenceId": 0, "position": {"x": 4.20, "y": 1.05, "theta": 1.57, "mapId": "warehouse-a"}, "actions": []},
      {"nodeId": "N13", "sequenceId": 2, "position": {"x": 4.20, "y": 6.30, "theta": 1.57, "mapId": "warehouse-a"}, "actions": [{"actionId": "a1", "actionType": "wait", "blockingType": "HARD", "actionParameters": {"seconds": 2}}]}
    ],
    "edges": [
      {"edgeId": "E12-13", "sequenceId": 1, "startNodeId": "N12", "endNodeId": "N13", "maxSpeed": 0.8, "maxRotationSpeed": 0.6}
    ]
  }
}
```
This mirrors VDA 5050's `order` schema directly (node/edge graph +
per-node actions), because it is a proven, minimal representation for
"go execute this route and do these things along the way," and it keeps
the ESP32's planner input format stable even if RMS/RCS logic changes.

### 2.4 `instantAction` (RCS → ESP32, higher priority than `order`)

```json
{
  "header": {"type": "instantAction", "headerId": 1043, "timestamp": "...", "version": "1.0.0", "vehicleId": "agv-07"},
  "payload": {"actions": [{"actionId": "ia-9", "actionType": "cancelOrder", "blockingType": "HARD", "actionParameters": {}}]}
}
```
Supported `actionType`s: `cancelOrder`, `startPause`, `stopPause`,
`emergencyStop`, `clearFault`, `requestFactsheet`.
The ESP32 processes `instantAction` frames out of order relative to
`order` frames (separate lock-free ring buffer) so an e-stop is never
queued behind a large order payload.

### 2.5 `state` (ESP32 → RCS, published at 5–10 Hz nominal, and on every
event/error transition)

```json
{
  "header": {"type": "state", "headerId": 88213, "timestamp": "...", "version": "1.0.0", "vehicleId": "agv-07"},
  "payload": {
    "orderId": "ord-2026-0007-014",
    "orderUpdateId": 0,
    "lastNodeId": "N12",
    "operatingMode": "AUTOMATIC",
    "position": {"x": 4.22, "y": 3.40, "theta": 1.55, "mapId": "warehouse-a", "positionSource": "EKF(IMU+UWB)", "covariance": [0.02, 0.02, 0.01]},
    "velocity": {"vx": 0.62, "vy": 0.0, "omega": 0.01},
    "battery": {"percentage": 76.5, "voltage": 25.1, "charging": false},
    "driveControllers": [
      {"id": "stm32-a", "linkOk": true, "faultCode": 0, "odriveVbus": 24.8, "odriveErrors": 0},
      {"id": "stm32-b", "linkOk": true, "faultCode": 0, "odriveVbus": 24.9, "odriveErrors": 0}
    ],
    "safetyState": "NORMAL",
    "errors": [],
    "actionStates": []
  }
}
```
Note that `driveControllers[]` surfaces STM32/ODrive health straight
through to the fleet UI — this is the field RMS uses to raise "vehicle
needs maintenance" alarms, and it is populated directly from the Hop 2
`MOTION_FB` frames (§3.4), i.e. Hop 1's `state` message is a superset
republish of what the ESP32 learns from the STM32s each cycle, decimated
to a UI-appropriate rate.

### 2.6 `connection` (both directions, heartbeat / liveness)
```json
{"header": {"type": "connection", "headerId": 1, "timestamp": "...", "version": "1.0.0", "vehicleId": "agv-07"}, "payload": {"connectionState": "ONLINE"}}
```
Sent every 1 s idle; if RCS misses 3 consecutive heartbeats it marks the
vehicle `CONNECTIONBROKEN` in RMS (vehicle itself continues its last order
segment and then executes STM32-local safe-stop per §3, independent of
this).

### 2.7 `telemetry` (ESP32 → RCS, optional high-rate raw stream for
diagnostics/replay, off by default, enabled per-vehicle for debugging)
Raw IMU/UWB samples, unfiltered — not used for control, only logging.

### 2.8 `ack`
Every `order`/`instantAction` is acknowledged (`headerId` echoed) before
`state` reflects it, so RCS can detect a dropped command vs. a rejected one.

---

## 3. Hop 2 — ESP32 ↔ STM32 (`stm_link`, UART, binary framed, real-time)

JSON is intentionally **not** used here: at a 1 kHz control loop on a
Cortex-M with FreeRTOS, JSON parsing is unnecessary CPU/jitter cost and
variable-length text parsing is a poor fit for a hard real-time budget. We
use a fixed binary frame with a length field and CRC16, modeled on the same
"envelope + typed payload" idea used at Hop 1, and on the framing pattern
used by other high-rate embedded telemetry protocols (e.g. MAVLink):

### 3.1 Frame format

```
byte 0      : STX            0xAA
byte 1      : VERSION        (protocol version, currently 1)
byte 2      : MSG_ID         (see table below)
byte 3      : SEQ            (rolling 0-255, per direction)
byte 4      : LEN            (payload length in bytes, 0-255)
byte 5..5+LEN-1 : PAYLOAD    (little-endian packed struct, see below)
last 2 bytes: CRC16-CCITT (poly 0x1021, init 0xFFFF) over bytes 1..(5+LEN-1)
```
STX is not included in the CRC so a receiver can resync on garbage.
Baud rate 921600 (configurable down to 115200 for noisy/long cable runs);
UART is 8N1, hardware flow control **off** (not available on all
ESP32/STM32 pin muxes used here — timeout-based framing is used instead).

### 3.2 Message IDs

| MSG_ID | Name | Direction | Rate |
|---|---|---|---|
| 0x01 | `MOTION_CMD` | ESP32 → STM32 | 50–100 Hz |
| 0x02 | `MOTION_FB` | STM32 → ESP32 | 100–200 Hz (or decimated) |
| 0x03 | `SAFETY_EVENT` | STM32 → ESP32 | on change (async) |
| 0x04 | `CONFIG_SET` | ESP32 → STM32 | on boot / reconfig, rare |
| 0x05 | `HEARTBEAT` | both | 10 Hz, fills gaps if no CMD/FB due |
| 0x06 | `ACK_NACK` | both | per received frame requiring ack (CONFIG_SET) |

### 3.3 `MOTION_CMD` payload (ESP32 → STM32), 17 bytes

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;   // ESP32 tick, for latency/jitter measurement
    float    velocity_mps;   // wheel/axle linear velocity setpoint
    float    torque_ff_nm;   // feedforward torque (0 if unused)
    uint8_t  mode;           // 0=IDLE 1=VELOCITY 2=POSITION 3=TORQUE 4=SAFE_STOP
    float    position_rad;   // used only if mode==POSITION
} motion_cmd_t;              // 4+4+4+1+4 = 17 bytes
```
The STM32's watchdog task requires a fresh `MOTION_CMD` (any `mode` value)
at least every `CMD_TIMEOUT_MS` (default 150 ms). Absence of a fresh frame
—independent of what the last received `mode` was— forces a local
decel-to-stop ramp and a transition to `mode=SAFE_STOP`, which is then
reported upward via `MOTION_FB.safety_state`. This is the concrete
mechanism behind the "WiFi/planner failure ≠ unsafe vehicle" guarantee in
`ARCHITECTURE.md` §4.

### 3.4 `MOTION_FB` payload (STM32 → ESP32), 29 bytes
```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float    velocity_actual_mps;
    int32_t  encoder_ticks;
    float    odrive_vbus_v;
    float    odrive_current_a;
    uint16_t odrive_error_flags;  // raw ODrive axis error bitmask
    uint8_t  safety_state;        // 0=NORMAL 1=WARN 2=SAFE_STOP 3=FAULT_LATCHED
    uint8_t  fault_code;          // local STM32 fault enum
} motion_fb_t;                    // 4+4+4+4+4+2+1+1 = 24 bytes (padded to 29 w/ reserved)
```

### 3.5 `SAFETY_EVENT` payload — pushed immediately (not waiting for the
next poll interval) whenever a hardware E-stop, bumper, wheel-lift switch,
or ODrive fault bit transitions, so the ESP32 (and therefore the fleet UI)
learns about it within one UART round trip instead of up to one FB period
later.

### 3.6 `CONFIG_SET` — sent once after link-up (and whenever `vehicle.yaml`
changes on the server and is propagated down): PID gains, wheel radius,
gear ratio, max velocity/accel limits, `CMD_TIMEOUT_MS`. STM32 stores it in
its local config struct (not persisted to flash by default, so a
misconfiguration cannot silently persist across the intended
"redeploy-from-server" workflow — flip `PERSIST_CONFIG=1` for depots that
want flash persistence instead).

### 3.7 Why CRC16 (not a simpler XOR/sum)
UART over a moving vehicle's cabling is exactly the kind of link where
burst errors happen (motor EMI, connector flex); CRC16-CCITT catches
those far more reliably than a checksum for a small, fixed extra cost that
is irrelevant at these frame sizes and this MCU class.

---

## 4. Hop 3 — STM32 ↔ ODrive (native ODrive ASCII protocol, unmodified)

We deliberately do not wrap or replace this: it is already a maintained,
documented, versioned protocol and re-implementing it would only add risk.

- Transport: UART, default 115200 baud (raise via `odrv0.config.uart_a_baudrate`
  if the link supports it; this repo configures 921600 to match Hop 2's
  budget headroom — see `firmware/stm32/protocol/odrive_uart.c`).
- Format: line-oriented ASCII, `COMMAND [ARGS] [*CHECKSUM] [; COMMENT]\n`.
  Checksum is an optional GCode-style XOR of all preceding bytes; if
  provided on the request, ODrive includes one on the response too.
- Commands used by this system:
  - `v <axis> <vel> <torque_ff>` — velocity setpoint (primary mode for
    differential-drive AGVs)
  - `p <axis> <pos> <vel_ff> <torque_ff>` — position setpoint (used for
    fine-docking moves)
  - `r <property>` — read a property, e.g. `r vbus_voltage`,
    `r axis0.motor.current_control.Iq_measured`, `r axis0.error`
  - `w <property> <value>` — write a config property (used only at
    commissioning time via `scripts/odrive_commission.py`, not in the
    real-time loop)
  - `ss` / motor-specific stop — used by the STM32 safety task on
    `SAFE_STOP`/`FAULT_LATCHED` transition, in addition to cutting the
    velocity command to zero, as defense in depth
- The STM32 is the **sole master**; it polls `r axis0.error`,
  `r vbus_voltage`, `r axis0.motor.current_control.Iq_measured` on a
  fixed schedule interleaved with `v`/`p` writes to stay within the 1 kHz
  budget, and republishes the results in the next `MOTION_FB` frame
  (§3.4). See `firmware/stm32/protocol/odrive_uart.c` for the exact
  interleave schedule.

---

## 5. Versioning & compatibility policy

- Hop 1 `header.version` is semver; RCS refuses an `order` from/to a
  vehicle whose reported major version mismatches, and instead surfaces a
  fleet alarm ("firmware upgrade required") — never silently reinterprets
  fields.
- Hop 2 `VERSION` byte is a simple monotonic integer; STM32 and ESP32
  firmware are built from the same `proto/stm_link.h` header (shared
  source of truth, see `proto/`), so a mismatch is a build-time, not
  run-time, concern in normal operation — the version byte exists purely
  to fail loudly if someone flashes mismatched images in the field.
- Hop 3 is whatever ODrive firmware version is flashed; `scripts/
  odrive_commission.py` checks `r hw_version` / `r fw_version_*` against a
  pinned known-good tuple at commissioning time.

## 6. Sequence diagram — one control tick, end to end

```
RCS               ESP32                  STM32-A         ODrive-A
 |--order-------->|                       |                |
 |                |--plan/trajectory-->   |                |
 |                |--MOTION_CMD(vel)----->|                |
 |                |                       |--"v 0 1.2 0"-->|
 |                |                       |<--"1.199"------|  (echo/ack per checksum use)
 |                |<--MOTION_FB-----------|                |
 |<--state--------|                       |                |
```
