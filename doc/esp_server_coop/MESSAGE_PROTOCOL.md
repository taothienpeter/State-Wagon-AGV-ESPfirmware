# Message Protocol Specification (CURRENT)

> **Cập nhật 2026-08-07.** Tài liệu này mô tả **giao thức hiện tại đang chạy** trong
> server (`app/protocol.py`, `app/tcp_server.py`, `app/fleet.py`, `app/registry.py`)
> và simulator (`tools/fake_esp32.py`). Đây là **single source of truth** cho wire format.
>
> ⚠️ Bản trước của file này mô tả thiết kế 3-hop kiểu VDA 5050 (Hop 1 TCP/JSON ↔ ESP32,
> Hop 2 UART/binary ↔ STM32 `stm_link`, Hop 3 ODrive ASCII) — **đó là tài liệu LỖI THỜI
> chưa từng được triển khai** trong repo này. Repo hiện chỉ có **Hop 1** (server ↔ ESP32)
> qua TCP/JSON. Hop 2/3 thuộc firmware `ESP32FirmwareV2` (ngoài phạm vi repo này).

## 1. Phạm vi & kiến trúc hiện tại

```
[Server (FastAPI/asyncio)] <==TCP/WiFi 7800==> [ESP32] (firmware, Phase 3)
        │  │                                    (simulator: tools/fake_esp32.py)
        │  ├── REST /api/*  (control + CRUD)   → Web UI
        │  └── WS /api/ws   (snapshot 0.25s)    → Web UI
```

- **Một connection TCP persistent mỗi AGV**, do **ESP32 (client) khởi tạo** tới server.
- Framing: `[u32BE length][UTF-8 JSON]`. Server chấp nhận tối đa **1 MiB**/message.
- Server multiplexes nhiều AGV trên cùng port qua trường `vehicleId`.
- Dev/lab chạy **plaintext TCP**; không có TLS/auth ở mức này (mạng nội bộ sân khấu).

## 2. Envelope (mọi message)

```json
{
  "type": "order | instantAction | state | connection | ack",
  "vehicleId": "agv-07",
  "payload": { "...": "type-specific, xem dưới" }
}
```

- `type` phải thuộc tập hợp lệ `ALLOWED_TYPES`; `vehicleId` là string không rỗng.
- Message lỗi hợp lệ bị `validate_envelope` từ chối (server log + bỏ qua, không ngắt kết nối).
- **Không** có `headerId`/`timestamp`/`version` bắt buộc (bản cũ dùng `header{}` — đã bỏ).

## 3. Các message — Server → ESP32

### 3.1 `order`
```json
{
  "type": "order",
  "vehicleId": "agv-07",
  "payload": {
    "orderId": "ord-1770000000-0001",
    "waypoints": [
      {"x": 4.2, "y": 1.05, "max_speed_mps": 0.8, "tolerance_m": 0.15}
    ]
  }
}
```
- `waypoints`: do MPG (`app/motion.py`) sinh — tối đa **200** (ngân sách ESP32 buffer 8192 B), mỗi điểm `{x, y, max_speed_mps}` (+ `tolerance_m` nếu move khai báo; giá trị dùng là **đồng nhất theo move**, không per-waypoint riêng).
- Tốc độ đã được clamp theo `capabilities.max_speed_mps` / `max_speed_limit` (1.2 m/s).

### 3.2 `instantAction`
```json
{
  "type": "instantAction",
  "vehicleId": "agv-07",
  "payload": {"actionType": "emergencyStop"}
}
```
- `actionType` hợp lệ (`ALLOWED_ACTIONS`): `emergencyStop`, `startPause`, `stopPause`, `clearFault`.
- Lệnh điều khiển khẩn (E-STOP) được gửi **ngay, không xếp hàng** sau order.

## 4. Các message — ESP32 → Server

### 4.1 `connection`
```json
{
  "type": "connection",
  "vehicleId": "agv-07",
  "payload": {"connectionState": "ONLINE", "capabilities": {"max_speed_mps": 0.9}}
}
```
- `connectionState`: `ONLINE` | `OFFLINE`.
- `capabilities` (tùy chọn): dùng để clamp tốc độ. Firmware cũ không gửi → fallback `max_speed_limit`.
- Gửi ngay khi kết nối (mark ONLINE).

### 4.2 `state` (tần suất ~8 Hz, và mỗi lần chuyển trạng thái/error)
```json
{
  "type": "state",
  "vehicleId": "agv-07",
  "payload": {
    "orderId": "ord-1770000000-0001",
    "orderState": "ACTIVE",
    "operatingMode": "AUTOMATIC",
    "safetyState": "NORMAL",
    "position": {"x": 4.22, "y": 3.40, "theta": 1.55},
    "velocity": {"vx": 0.62, "vy": 0.0, "omega": 0.01},
    "battery": {"percentage": 76.5, "voltage": 25.1, "charging": false},
    "imu": {"accel": [0, 0, 9.8], "gyro": [0, 0, 0]},
    "uwb": {"anchors": 3, "quality": 0.92},
    "driveControllers": [
      {"id": "stm32-a", "linkOk": true, "faultCode": 0, "odriveVbus": 24.8, "odriveErrors": 0}
    ],
    "errors": []
  }
}
```
- Server (`app/registry.py`) parse/giữ: `orderId`, `orderState` (IDLE|ACTIVE|PAUSED|COMPLETED|FAILED|ESTOP), `position`, `velocity`, `battery`, `safetyState`, `operatingMode`; lưu nguyên payload vào `session.state`.
- **`orderState` là nguồn phụ** cho hoàn thành cue — nguồn chính là **poll position** (tolerance). `orderState=COMPLETED` chỉ dùng khi thiếu position (xem `decisions.md` D9).
- **Theo dõi order:** khi `orderId` đổi → server push 1 bản ghi vào `recent_orders` (≤10): `{orderId, orderState, ack_status, cueId, ts}`. `cueId` do server gắn khi dispatch show.
- `state` từ simulator bổ sung `imu`/`uwb`/`driveControllers`/`errors` (hiển thị, không dùng điều khiển).

### 4.3 `ack` (xác nhận đã nhận order)
```json
{
  "type": "ack",
  "vehicleId": "agv-07",
  "payload": {"orderId": "ord-1770000000-0001", "status": "ACCEPTED", "reason": "optional"}
}
```
- `status`: `ACCEPTED` | `REJECTED` (bắt buộc); `orderId` bắt buộc; `reason` tùy chọn (REJECTED).
- Server khớp `orderId` với `session.order_id` — **ack của order cũ không ghi đè order hiện tại**.

## 5. Xác nhận đơn hàng (ack / offline)

- Cờ `order_ack_enabled` (config, mặc định **`false`** vì firmware chưa gửi ack):
  - `false` → `FleetService.send_order` trả `SENT` ngay sau khi ghi (không chờ ack); cue hoàn thành dựa **poll position**.
  - `true` → server chờ ack trong `order_ack_timeout_s` (2.0s, poll `asyncio.sleep(0.02)`):
    - ack `ACCEPTED` → `ACCEPTED` (ok).
    - ack `REJECTED` (kèm reason) → `REJECTED` (không ok).
    - hết timeout → `NO_ACK` (không ok).
- **Offline:** server đánh dấu OFFLINE nếu không nhận `state` trong `offline_timeout_s` (3s); sweep task mỗi 1s. ESP32 tiếp tục an toàn nội bộ riêng.

## 6. REST API + WebSocket (server → Web UI)

### REST (`/api`, FastAPI)
| Method | Path | Chức năng |
|---|---|---|
| GET | `/api/health` | health |
| GET | `/api/agvs` | `registry.snapshot()` (mỗi xe kèm `order` + `recent_orders`) |
| POST | `/api/estop` · `/api/estop/release` | E-STOP toàn cục / nhả |
| POST | `/api/order` | order debug/manual (chặn 409 nếu AGV thuộc cue chạy) |
| POST | `/api/agvs/{id}/action` | instantAction 1 xe (chặn pause/resume khi AGV thuộc cue) |
| GET/POST/PUT/DELETE | `/api/shows...` | CRUD show (schema v2: scenes/cues/moves) |
| POST | `/api/shows/{id}/load` · `/unload` | nạp/dỡ show runtime |
| POST | `/api/shows/{id}/go/{i}` · `/pause` · `/resume` · `/skip/{i}` · `/clear-fault` | điều khiển show |
| POST | `/api/go` | GO tổng (bắt đầu / cue kế) |
| GET | `/api/runtime` · `/api/stage` | trạng thái runtime + sân khấu |
| WS | `/api/ws` | snapshot realtime |

### WS snapshot (broadcast 0.25 s)
```json
{
  "type": "snapshot",
  "ts": 1770000000.123,
  "agvs": [ { "vehicleId": "agv-07", "online": true, "position": {...}, "velocity": {...},
              "orderState": "ACTIVE", "battery": {...}, "safetyState": "NORMAL",
              "operatingMode": "AUTOMATIC", "capabilities": {...},
              "order": {"orderId": "...", "orderState": "...", "ack_status": "ACCEPTED", "cueId": "..."},
              "recent_orders": [ ... ] } ],
  "runtime": { "active": true, "showId": "...", "name": "...", "state": "RUNNING",
               "cursor": 0, "activeCueIndex": 0, "cues": [ {"index":0,"cueId":"...","name":"...",
               "status":"RUNNING","trigger":"MANUAL","blocking":true,"timeout_ms":0,"elapsedMs":1234,"error":null} ] },
  "estop": {"active": false, "triggeredAt": null, "by": null},
  "stage": {"width_m": 12.0, "height_m": 8.0}
}
```

## 7. Phiên bản & tương thích

- Định dạng hiện tại là **de facto** (repo này). Không có negotiation version ở wire; khớp nhau theo code.
- **Thực trạng firmware ESP32 (Phase 3) hiện có 3 lỗ hổng so với spec này** — xem chi tiết +
  hướng dẫn co-op tại `Plans/StageWagonServer/ESP32_CONNECTION_GUIDE.md`:
  1. **Chưa gửi `orderState`** trong `state` → server chỉ dựa poll position để hoàn thành cue.
  2. **Chưa gửi `ack`** → `order_ack_enabled` đang mặc định `false`.
  3. **`operatingMode` dùng `EMERGENCY`** khi ESTOP — ngoài tập `ALLOWED_OPERATING_MODES`.
  - Gotcha: firmware `loadWaypoints` **thay thế** trajectory (không queue) — server dispatch cùng AGV
    theo hướng tuần tự để tránh gửi order mới khi AGV đang chạy order cũ.
- Khi hoàn thiện **Phase 3**: firmware phải gửi đúng `ack` (`{orderId,status}`), `state` (có
  `orderId`/`orderState`/`position`), và chấp nhận `order` với `waypoints[]`. Sau đó bật `order_ack_enabled: true`.

## 8. Danh sách field/giới hạn (tham chiếu nhanh)

| Hạng mục | Giá trị |
|---|---|
| Framing | `[u32BE len][UTF-8 JSON]` |
| TCP port | 7800 (config `port`) |
| Payload max (server) | 1 MiB (`MAX_PAYLOAD_BYTES`) |
| Waypoint tối đa / order | 200 |
| Tần suất state | ~8 Hz (simulator), có thể thấp hơn firmware thật |
| Offline timeout | 3 s (`offline_timeout_s`), sweep 1 s |
| `order_ack_enabled` | mặc định `false` |
| `order_ack_timeout_s` | 2.0 |
| Tốc độ tối đa | `capabilities.max_speed_mps` / `max_speed_limit_mps` (1.2) |
| Sân khấu mặc định | 12 × 8 m (config `stage.width_m/height_m`) |

## 9. Thay đổi so với bản cũ (2026-08-07)

- **Bỏ** envelope `{header:{type,headerId,timestamp,version,vehicleId}}` → envelope phẳng `{type, vehicleId, payload}`.
- **Bỏ** schema `order` theo `nodes[]`/`edges[]` → `waypoints[]`.
- **Bỏ** `instantAction.actions[]` → `{actionType}` scalar; chỉ 4 action hợp lệ (không có `cancelOrder`/`requestFactsheet`).
- **Bỏ** loại `telemetry`.
- **Bỏ** toàn bộ Hop 2 (`stm_link` UART/binary) và Hop 3 (ODrive ASCII) khỏi spec này — thuộc firmware ngoài repo.
- Giữ: khái niệm `connection`/`state`/`order`/`instantAction`/`ack` + confirmed dispatch (cờ `order_ack_enabled`) + `recent_orders`.
