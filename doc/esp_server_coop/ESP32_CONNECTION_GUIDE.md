# Hướng dẫn kỹ thuật: Kết nối ESP32 ↔ Server (StageWagonServer)

> **Mục đích:** tài liệu này dành cho **agent phát triển firmware ESP32** (repo `ESP32FirmwareV2`)
> và **agent phát triển server** (repo `StageWagonServer`) để **co-op hiệu quả** quanh mối nối TCP.
> Đây là **contract** giữa hai bên: cách kết nối, framing, message, và các quy tắc bắt buộc.
>
> **Nguồn sự thật (server):** `app/protocol.py`, `app/tcp_server.py`, `app/registry.py`,
> `app/fleet.py`, `app/config.py`, `app/motion.py`.
> **Nguồn sự thật (firmware hiện có):** `ESP32FirmwareV2/main/main.cpp`,
> `components/comm/hop1_client.cpp`, `components/trajectory/trajectory_gen.*`.
> **Simulator (tham chiếu client):** `tools/fake_esp32.py`.
>
> Xem thêm: `Plans/StageWagonServer/MESSAGE_PROTOCOL.md` (spec wire format chi tiết).

---

## 1. Kiến trúc tổng quan

```
 [Web UI (React/Vite)]
         │  WS /api/ws  (snapshot 0.25s)   +   REST /api/*
         ▼
 [Server  StageWagonServer  (FastAPI + asyncio)]
         │
         │  TCP plaintext  :7800  (config `port`, mặc định 7800)
         ▼
 [ESP32 firmware]  ←──  client kết nối tới server (TCP outbound, persistent)
         │
         ▼
 [STM32] via UART (921600 baud, binary — NGOÀI phạm vi tài liệu này)
```

- **ESP32 là TCP CLIENT** — khởi tạo kết nối tới server, không phải server kết nối tới ESP32.
- **Một connection persistent cho mỗi AGV**, giữ suốt vòng đời; nhiều AGV multiplex qua trường `vehicleId`.
- **Plaintext TCP**, không TLS/auth ở mức này (mạng nội bộ sân khấu).
- Server vừa nhận `state`/`connection`/`ack`, vừa **ghi `order`/`instantAction` xuống writer** của connection tương ứng.

---

## 2. Cổng & cấu hình

| Hạng mục | Giá trị | Config (server) | Firmware (config.h) |
|---|---|---|---|
| TCP port | **7800** | `port` (`config.json` / env `PORT`) | `AGV_DEFAULT_SERVER_PORT` |
| HTTP/WS port | 8000 | `http_port` | — |
| Server host | IP máy chạy server | `host` (`0.0.0.0`) | `AGV_DEFAULT_SERVER_HOST` |
| Tần suất publish state | **8 Hz** | — | `AGV_STATE_PUBLISH_HZ` |
| Offline timeout | **3 s** | `offline_timeout_s` | — |
| Order ack | mặc định **tắt** | `order_ack_enabled` | — |

> Dev thường dùng `python script/run_dev.py` (supervisor) hoặc `python -m app.main`; có thể
> override cổng bằng `--server-port` / `--http-port` để tránh xung đột.

---

## 3. Vòng đời kết nối

1. **Boot ESP32** → kết nối WiFi (timeout `AGV_WIFI_CONNECT_TIMEOUT_MS` = 15 s).
2. **TCP connect** tới `host:port`. Kết nối thành công.
3. **Gửi `connection`** với `connectionState: "ONLINE"` ngay (để server đánh dấu ONLINE, đăng ký session). *Không gửi `connection` trước thì server vẫn đăng ký session khi nhận message đầu, nhưng phải gửi để bật `online` đúng lúc.*
4. **Vòng lặp duplex persistent**:
   - Server → ESP32: `order`, `instantAction`.
   - ESP32 → Server: `state` (~8 Hz), `ack` (nếu bật).
5. **Mất kết nối** → tái kết nối với **backoff** 500 ms → 15 s (tăng dần), lặp lại từ bước 2.
6. Server đánh dấu OFFLINE nếu không nhận `state` trong `offline_timeout_s` (3 s), sweep mỗi 1 s.
   ESP32 tự tiếp tục an toàn nội bộ riêng (trajectory/xử lý lỗi) khi mất mạng.

---

## 4. Framing (bắt buộc, cả 2 chiều)

Mọi message là một **frame**:

```
[u32 BIG-ENDIAN length][UTF-8 JSON body]
```

- `length` = số byte của phần JSON body (không gồm 4 byte header), **big-endian (network order)**.
- Server chấp nhận tối đa **1 MiB**/message (`MAX_PAYLOAD_BYTES`).
- Đọc từng chunk TCP, gộp vào buffer, parse từng frame theo length prefix; frame lở dang giữ lại.
- JSON phải hợp lệ; server `validate_envelope` sẽ **bỏ qua** message không hợp lệ (log, không ngắt kết nối).

Ví dụ một `state` mã hóa:
```
[00 00 01 A0][{"type":"state","vehicleId":"agv-07","payload":{...}}]
```
(ở đây `0x1A0` = 416 byte JSON).

---

## 5. Envelope (mọi message)

```json
{
  "type": "order | instantAction | state | connection | ack",
  "vehicleId": "agv-07",
  "payload": { "...": "type-specific" }
}
```

- `type` phải thuộc tập hợp lệ: `order`, `instantAction`, `state`, `connection`, `ack`.
- `vehicleId`: string **không rỗng**; phải trùng ID xe thật để server gắn đúng session/order.
- **Không có** `header{}`/`headerId`/`timestamp`/`version` (bản cũ có — đã bỏ).

---

## 6. Server → ESP32 (ESP32 phải đọc & xử lý)

### 6.1 `order` — ra lệnh di chuyển

```json
{
  "type": "order",
  "vehicleId": "agv-07",
  "payload": {
    "orderId": "ord-1770000000-0001",
    "waypoints": [
      { "x": 4.2, "y": 1.05, "max_speed_mps": 0.8, "tolerance_m": 0.15 },
      { "x": 6.0, "y": 1.05, "max_speed_mps": 0.8, "tolerance_m": 0.15 }
    ]
  }
}
```

**Quy tắc firmware phải tuân:**
- `waypoints[]` là danh sách điểm **theo thứ tự** A→B→C… ESP32 chạy lần lượt.
- Mỗi điểm: `x`, `y` (mét), `max_speed_mps` (tốc độ tối đa cho đoạn), `tolerance_m` (dung sai tới điểm, đồng nhất theo move).
- Tối đa **200 waypoint**/order (ngân sách buffer 8192 B ESP32).
- Tốc độ đã được server clamp theo `capabilities.max_speed_mps` / `max_speed_limit` (1.2 m/s).
- **Số `orderId` phải lưu lại** để: (a) gắn vào `state.orderId`, (b) phản hồi `ack`.

> ⚠️ **CẢNH BÁO TRAJECTORY — không có hàng đợi:** `g_trajectory.loadWaypoints(wps)` **THAY THẾ**
> toàn bộ waypoint đang chạy (reset `active_index_=0`). Nếu server gửi order mới khi AGV đang chạy
> order cũ, order cũ **bị cắt ngang**. Server thiết kế để **không** gửi order cùng AGV chồng nhau
> (dispatch tuần tự), nhưng firmware nên xử lý thứ tự an toàn (chờ ACTIVE→COMPLETED) nếu có hàng đợi nội bộ.

### 6.2 `instantAction` — hành động tức thời

```json
{
  "type": "instantAction",
  "vehicleId": "agv-07",
  "payload": { "actionType": "emergencyStop" }
}
```

`actionType` hợp lệ (`ALLOWED_ACTIONS`):
| actionType | Nghĩa | Firmware nên làm |
|---|---|---|
| `emergencyStop` | E-STOP | dừng khẩn, set ESTOP, giữ an toàn |
| `startPause` | tạm dừng | `trajectory.pause()` |
| `stopPause` | tiếp tục | `trajectory.resume()` |
| `clearFault` | nhả lỗi | `trajectory.clearEstop()` |

Gửi **ngay, không xếp hàng sau order** (ưu tiên khẩn).

---

## 7. ESP32 → Server (ESP32 phải gửi)

### 7.1 `connection`

```json
{
  "type": "connection",
  "vehicleId": "agv-07",
  "payload": {
    "connectionState": "ONLINE",
    "capabilities": { "max_speed_mps": 0.9 }
  }
}
```

- `connectionState`: `ONLINE` | `OFFLINE`.
- `capabilities` (tùy chọn): server dùng để clamp tốc độ. Gửi `max_speed_mps` nếu firmware khai được; không gửi thì server fallback `max_speed_limit` (1.2).
- Gửi ngay khi connect.

### 7.2 `state` (bắt buộc, ~8 Hz + mỗi lần chuyển trạng thái/error)

```json
{
  "type": "state",
  "vehicleId": "agv-07",
  "payload": {
    "orderId": "ord-1770000000-0001",
    "orderState": "ACTIVE",
    "operatingMode": "AUTOMATIC",
    "safetyState": "NORMAL",
    "position": { "x": 4.22, "y": 3.40, "theta": 1.55 },
    "velocity": { "vx": 0.62, "vy": 0.0, "omega": 0.01 },
    "battery": { "percentage": 76.5, "voltage": 25.1, "charging": false },
    "imu": { "accel": [0,0,9.8], "gyro": [0,0,0] },
    "uwb": { "anchors": 3, "quality": 0.92 },
    "driveControllers": [
      { "id": "stm32-main", "linkOk": true, "faultCode": 0,
        "odriveVbus": 24.8, "odriveErrors": 0 }
    ],
    "errors": []
  }
}
```

**Server parse/giữ các field:** `orderId`, `orderState`, `position`, `velocity`, `battery`,
`safetyState`, `operatingMode`; lưu nguyên payload vào `session.state`.

**Field quan trọng nhất cho hoàn thành cue — `orderState`:**
- Giá trị hợp lệ: `IDLE | ACTIVE | PAUSED | COMPLETED | FAILED | ESTOP`.
- Server **ưu tiên poll vị trí** (so với `tolerance_m`) để xác định đã tới đích; `orderState=COMPLETED`
  chỉ là **nguồn phụ** khi thiếu position (xem `decisions.md` D9).
- ⚠️ **Firmware hiện tại CHƯA gửi `orderState`** trong `state` (`main.cpp` statePublishTaskFunc chỉ
  gửi `orderId`/`operatingMode`/`safetyState`/`position`/`velocity`/...). Agent firmware **nên bổ sung
  `orderState`** (map từ `TrajectoryGen::status()`: `IDLE`→`IDLE`, `ACTIVE`→`ACTIVE`,
  `PAUSED`→`PAUSED`, `COMPLETE`→`COMPLETED`, `ESTOP`→`ESTOP`, kèm `FAILED` khi có lỗi) để server
  theo dõi order chính xác.

- **Theo dõi order (`recent_orders`):** khi `orderId` trong `state` ĐỔI → server thêm 1 bản ghi
  `{orderId, orderState, ack_status, cueId, ts}` (≤ 10). `cueId` server tự gắn khi dispatch show.
  → Firmware phải **set `orderId` = order đang chạy** trong mỗi `state`.

**Lưu ý `operatingMode`:** server `ALLOWED_OPERATING_MODES` gồm `AUTOMATIC|SEMIAUTOMATIC|MANUAL|
SERVICE|TELEOPERATED|UNKNOWN`. Firmware hiện gửi `EMERGENCY`/`MANUAL` khi ESTOP — **không** nằm trong
tập hợp lệ của server, nhưng vì `state` không được deep-validate nên không gây lỗi. Muốn chuẩn hóa,
nên gửi `EMERGENCY`→`MANUAL` hoặc thống nhất với server (thêm `EMERGENCY` vào `ALLOWED_OPERATING_MODES`).

### 7.3 `ack` (xác nhận đã nhận order — chỉ cần khi bật `order_ack_enabled`)

```json
{
  "type": "ack",
  "vehicleId": "agv-07",
  "payload": { "orderId": "ord-1770000000-0001", "status": "ACCEPTED", "reason": "optional" }
}
```

- `status`: `ACCEPTED` | `REJECTED` (bắt buộc); `orderId` bắt buộc; `reason` tùy chọn khi `REJECTED`.
- Server khớp `orderId` với `session.order_id` — ack của order cũ không ghi đè order hiện tại.
- ⚠️ **Firmware hiện tại CHƯA gửi `ack`** (`hop1_client.cpp` chỉ xử lý order/instantAction). Khi agent
  firmware thêm `ack`, server mới có thể bật `order_ack_enabled: true`.

---

## 8. Xác nhận đơn hàng (ack) & hoàn thành cue

| Cờ | Mặc định | Ý nghĩa |
|---|---|---|
| `order_ack_enabled` | `false` | `false`: `send_order` trả `SENT` ngay, hoàn thành dựa **poll position**. `true`: chờ ack trong `order_ack_timeout_s` (2.0 s). |
| `order_ack_timeout_s` | 2.0 | Thời gian chờ ack khi bật cờ. |
| `offline_timeout_s` | 3.0 | Ngưỡng không nhận `state` → OFFLINE. |

- Khi `order_ack_enabled=false` (hiện tại): server không chờ ack, chỉ dựa position để biết tới đích.
- Khi bật `true`: ack `ACCEPTED` → ok; `REJECTED` → không ok; hết timeout → `NO_ACK` (không ok).

---

## 9. Các quy tắc / gotcha firmware bắt buộc

1. **Framing đúng `[u32BE len][JSON]`** cả 2 chiều — sai là server bỏ message.
2. **`vehicleId` phải khớp** ID thật của xe — server gắn session theo field này.
3. **Connection persistent**; reconnect backoff 500 ms→15 s.
4. **Gửi `state` liên tục ~8 Hz** kể cả khi IDLE (để server không đánh OFFLINE trong 3 s).
5. **Set `orderId` + `orderState`** trong mọi `state` khi có order.
6. **`loadWaypoints` THAY THẾ** trajectory — không có hàng đợi; đừng nhận 2 order cùng lúc nếu chưa hỗ trợ queue.
7. **Bổ sung `ack`** nếu muốn bật `order_ack_enabled` (xác nhận đơn hàng chặt chẽ).
8. **Tôn trọng `max_speed_mps`/`tolerance_m`** mỗi waypoint.
9. **E-STOP phải dừng ngay và giữ an toàn**; `clearFault` mới cho chạy lại.
10. **Không gửi `orderState=COMPLETED`** cho tới khi thực sự tới waypoint cuối.

---

## 10. Tham chiếu nhanh field/giới hạn

| Hạng mục | Giá trị |
|---|---|
| Framing | `[u32BE len][UTF-8 JSON]` |
| TCP port | 7800 (config `port`) |
| Payload max (server) | 1 MiB |
| Waypoint tối đa / order | 200 |
| Tần suất state | ~8 Hz |
| Offline timeout | 3 s (sweep 1 s) |
| `order_ack_enabled` | mặc định `false` |
| `order_ack_timeout_s` | 2.0 |
| Tốc độ tối đa | `capabilities.max_speed_mps` / `max_speed_limit` (1.2 m/s) |
| Sân khấu mặc định | 12 × 8 m (`stage.width_m/height_m`) |
| Buffer ESP32 | 8192 B (waypoint) |

---

## 11. Tham chiếu triển khai

- **Simulator client (Python) — dùng làm mẫu đúng nhất:** `tools/fake_esp32.py`
  - `connect()` → `_send_connection()` → vòng lặp `_send_state()` (8 Hz) + `_receive_loop()`.
  - `_handle_order(payload)` → set `current_order_id`, `waypoints`, gửi ack, `_next_waypoint()`.
  - `_send_ack(status, reason)`.
- **Firmware hiện có (ESP-IDF/C++):**
  - `components/comm/hop1_client.cpp`: kết nối, framing, gửi `connection`/`state`, nhận `order`/`instantAction`, reconnect backoff.
  - `main/main.cpp`: `statePublishTaskFunc` (~8 Hz), `onOrderReceived` → `g_trajectory.loadWaypoints(wps)`.
  - `components/trajectory/trajectory_gen.{h,cpp}`: `loadWaypoints`, `tick`, `pause/resume/emergencyStop/clearEstop`, `status()`.

---

## 12. Checklist để co-op thành công

Khi agent firmware giao việc / kiểm thử, đối chiếu checklist này:

- [ ] Kết nối TCP tới `host:7800`, gửi `connection` `ONLINE`.
- [ ] Gửi `state` ~8 Hz, có `orderId` + `orderState`.
- [ ] Nhận `order`, chạy waypoints theo thứ tự, tôn trọng `max_speed_mps`/`tolerance_m`.
- [ ] Báo `orderState=COMPLETED` khi tới waypoint cuối (hoặc để server poll position).
- [ ] Xử lý `instantAction` (ít nhất `emergencyStop` dừng ngay).
- [ ] Reconnect backoff khi mất mạng; server tự đánh OFFLINE.
- [ ] (Tùy chọn, khi bật ack) gửi `ack {orderId, status}`.

---

## 13. Đề xuất cho agent firmware (tình trạng hiện tại)

Kế thừa code hiện có và bổ sung **3 mảnh còn thiếu** để khớp contract:
1. **Gửi `orderState`** trong `state` (map từ `TrajectoryGen::status()`).
2. **Gửi `ack`** khi nhận `order` (để có thể bật `order_ack_enabled`).
3. **Chuẩn hóa `operatingMode`** (không dùng `EMERGENCY`; dùng `MANUAL`/`AUTOMATIC` hoặc phối hợp
   server thêm `EMERGENCY` vào `ALLOWED_OPERATING_MODES`).
