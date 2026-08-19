# StageWagonServer — Bộ Kế Hoạch Triển Khai Chi Tiết

> **Vị trí:** Bộ plan chính thức nằm tại `StageWagonServer\Plans\` (mục tiêu triển khai). Bản sao lưu trữ trong `.kilo\plans\` (cấu hình quyền) được giữ để tham chiếu; khi sửa, ưu tiên sửa bản tại `StageWagonServer\Plans\`.

Bộ plan gồm **8 file**: 1 README (chỉ mục + quan hệ + contract chuẩn) + 4 plan Server + 3 plan Web UI. Mỗi plan là một **đơn vị thực thi độc lập** nhưng khai báo rõ phụ thuộc lẫn nhau qua contract.

---

## 1. QUYẾT ĐỊNH ĐÃ CHỐT (làm chuẩn cho mọi plan)

| # | Quyết định | Giá trị |
|---|-----------|---------|
| 1 | Tech stack Web UI | Vite + React + TS, project `web/`, mount `/studio`; giữ `static/` cũ ở `/` |
| 2 | Phục vụ frontend | Dev: Vite proxy `/api`→`:8000`; Thật: `npm run build` → FastAPI mount `web/dist` tại `/studio` |
| 3 | Phạm vi firmware | **Server trước, firmware sau.** Cờ `order_ack_enabled` mặc định `False`; `fake_esp32.py` mô phỏng ack |
| 4 | Hoàn thành cue | Poll **position** (registry 8Hz) là nguồn chính; `orderState=COMPLETED` là phụ khi firmware có |
| 5 | Khôi phục lỗi | `clear_fault()` → resume từ cue lỗi; `clear_estop()` → reset từ đầu |
| 6 | Received cues | Từ phản hồi liên tục ESP32 (`order` + `recent_orders[]`) trong WS snapshot; không cần REST orders log |
| 7 | Timeline | Auto-layout (clip nối tiếp theo trigger/blocking) |
| 8 | Flow Graph | Hiển thị theo dữ liệu; backend vẫn **1 cue RUNNING**/lúc |
| 9 | Bo góc | `corner_blend_m` một giá trị/cue |
| 10 | Rehearsal loop | Hoãn |

---

## 2. BẢN ĐỒ QUAN HỆ GIỮA CÁC MODULE (cross-module)

```
                        ┌─────────────────────────────────────────────┐
                        │                 WEB UI (React/Vite)          │
                        │   webui/01 Cue Editor   webui/02 Live Monitor│
                        │   webui/03 AGV Status    (1 shared store)    │
                        └───────▲──────────────────▲──────────────────┘
                                │ REST /api/*       │ WS /api/ws (snapshot)
                        ┌───────┴──────────────────┴──────────────────┐
                        │            server/04 API+TCP+Storage         │
                        │   routes.py ──► ws.py ──► main.py ──► tcp_server │
                        └───────▲──────────────────▲──────────────────┘
                        ┌───────┴──────────────────┴──────────────────┐
                        │        server/01 Show Manager (state machine)│
                        │            (consumer của 02, 03)             │
                        └───────▲──────────────────▲──────────────────┘
                  ┌─────────────┴──┐          ┌───┴─────────────┐
        ┌─────────┴────────┐ ┌─────┴─────┐  ┌─┴──────────────┐
        │ server/02 Fleet   │ │           │  │ server/03 MPG  │
        │ (send/ack)        │ │           │  │ + Registry     │
        └─────────▲────────┘ └───────────┘  └─▲──────────────┘
                  │ (gửi order, chờ ack)       │ (contract: protocol, raw move)
        ┌─────────┴────────────────────────────┴────────────────┐
        │                 server/04 protocol.py  (contract chuẩn)│
        │   ack · state · connection · order · instantAction     │
        └───────────────────────────────▲───────────────────────┘
                                        │ TCP 7800
                              ┌─────────┴──────────┐
                              │ ESP32 (firmware, sau)│
                              └─────────────────────┘
```

**Chiều phụ thuộc triển khai (đảo ngược):** `protocol (04)` ← `registry+mpg (03)` ← `fleet (02)` ← `show (01)` ← `api+tcp (04)` ← `webui`.

---

## 3. CONTRACT CHUẨN GIỮA CÁC MODULE (single source of truth)

Đây là contract mà mọi module phải tuân theo; được định nghĩa **đầy đủ và không mập mờ** để không có 2 nơi hiểu khác nhau.

### 3.1 Protocol wire (server/04 — `protocol.py`)
```
Envelope:  {"type", "vehicleId", "payload", "headerId"?}
Frame:     [4-byte BE length][UTF-8 JSON], max 1 MiB (server) / 8192 (ESP32)
Types:     order, instantAction, state, connection, ack
order:        payload {orderId, waypoints:[{x,y,max_speed_mps,tolerance_m?}]}
instantAction:payload {actionType: emergencyStop|startPause|stopPause|clearFault}
state:        payload {orderId?, orderState?, position, velocity, battery, safetyState, operatingMode, driveControllers?}
connection:   payload {connectionState, capabilities?}
ack:          payload {orderId, status: ACCEPTED|REJECTED, reason?}
```

### 3.2 Show schema v2 (disk — `data/shows/{id}.json`)
```
show: {showId, name, version:2, cues[]}
cue:  {cueId, name, trigger:MANUAL|AUTO_FOLLOW, after?:cueId,
       blocking:bool=true, timeout_ms:int=0, moves[]}
move: {vehicleId, target:{x,y}, max_speed_mps, tolerance_m, corner_blend_m=0}
Back-compat: reads moves[].waypoints → target = điểm cuối.
```

### 3.3 RuntimeStatus (server/01 → server/04 → WS → WebUI)
```
{active, showId, name, state, cursor, activeCueIndex,
 cues:[{index, cueId, name, status, trigger, blocking, timeout_ms,
        elapsedMs?, error?}]}
CueStatus: PENDING|READY|RUNNING|COMPLETED|TIMEOUT|ERROR|SKIPPED|ABORTED
ShowState: STOPPED|RUNNING|PAUSED|FAULT|ESTOP|COMPLETED
```

### 3.4 WS snapshot (server/04 → WebUI)
```
{type:"snapshot", ts,
 agvs:[{vehicleId, online, position, velocity, orderState, safetyState,
       operatingMode, battery, capabilities, order?, recent_orders[]}],
 runtime:<RuntimeStatus>, estop:{active}, stage:{width_m, height_m}}
```

### 3.5 OrderResult (server/02 → server/01)
```
OrderResult{orderId, status, ack_status, error}
status:  ACCEPTED | REJECTED | NO_ACK | SENT
ack_status: "ACCEPTED"|"REJECTED"|None
```

### 3.6 RawMove → Waypoint[] (server/03 → server/01)
```
RawMove{vehicle_id, points:[{x,y}], max_speed_mps, tolerance_m, corner_blend_m}
plan(raw, capabilities, stage) → [{x,y,max_speed_mps,tolerance_m?}]  ≤200
```

### 3.7 REST API (server/04 → WebUI)
Xem chi tiết trong plan `server/04`, mục "REST API surface".

---

## 4. CẤU TRÚC FILE

```
Plans/
├── README.md                          ← file này (chỉ mục + contract chuẩn)
├── server/
│   ├── 01-show-manager.md             ← state machine 2 tầng + supervisor
│   ├── 02-fleet-service.md            ← confirmed dispatch + ack + instantAction
│   ├── 03-mpg-vehicle-registry.md     ← MotionPlanner + VehicleRegistry
│   └── 04-api-tcp-storage.md          ← protocol + tcp + storage + REST/WS + mount
├── webui/
│   ├── 01-cue-editor.md               ← Cue Table + Timeline + Flow Graph
│   ├── 02-live-monitor.md             ← viewport sân khấu + transport
│   └── 03-agv-status-panels.md        ← AGV card + received cues + inspector
├── STATUS.md · progress.md · decisions.md ← memory (snapshot + tiến độ + quyết định)
└── ../MESSAGE_PROTOCOL.md             ← SPEC giao thức HIỆN TẠI (Plans/, xem §7)
```

> **Ghi chú (2026-08-07):** `MESSAGE_PROTOCOL.md` là **spec giao thức hiện tại** (khớp `app/protocol.py` + REST/WS): envelope phẳng `{type, vehicleId, payload}`, order `waypoints[]`, instantAction `{actionType}`, ack `{orderId,status}`, offline 3s, confirmed dispatch qua cờ `order_ack_enabled`. Bản trước (3-hop VDA-5050-flavored: `header{}`, `nodes/edges`, `stm_link`, ODrive) là **LỖI THỜI — đã bỏ** (xem `decisions.md` D44).

---

## 5. THỨ TỰ TRIỂN KHAI & ĐƠN VỊ GIAO NHẬN

| Phase | Nội dung | Điều kiện "Done" (Definition of Done) |
|-------|----------|-----------------------------------------|
| 1 | `server/03` → `server/02` → `server/01` → `server/04` | Backend pass toàn bộ unit test; WS snapshot đầy đủ; fake_esp32 chạy end-to-end |
| 2 | `webui/02` → `webui/03` → `webui/01` | 3 view render từ snapshot thật; transport thao tác được |
| 3 | ESP32 firmware: buffer 8192 + sendAck + orderState | Bật `order_ack_enabled=true`; integration với firmware thật |

**Ghi chú chung:** đơn vị **mét**; test: `python -m pytest tests/ -q`; build web: `cd web && npm run build`.

---

## 6. ĐÁNH GIÁ TỔNG HỢP (SCALE 1–5)

Thang đánh giá: **1** = rất đơn giản · **5** = rất phức tạp. Cột "Mức sẵn sàng triển khai" phản ánh mức độ rủi ro/chuẩn bị.

| Plan | Độ phức tạp (1–5) | Rủi ro chính | Mức sẵn sàng |
|------|:---:|--------------|:---:|
| `server/03` MPG + Registry | ★★★☆☆ (3) | Toán bo góc (`_blend_corner`), ack_status so khớp | Cao — nền tảng, ít phụ thuộc |
| `server/02` Fleet Service | ★★☆☆☆ (2) | Poll ack giữ event loop; cờ config | Cao |
| `server/01` Show Manager | ★★★★☆ (4) | Race supervisor × API; timeout/offline/estop đan xen | Trung bình — cần test kỹ |
| `server/04` API + TCP + Storage | ★★★☆☆ (3) | Wiring estop→show; on_change; mount | Cao — đa số gọi module khác |
| `webui/02` Live Monitor | ★★★☆☆ (3) | Canvas tọa độ (y-flip), zoom/pan, trail | Trung bình |
| `webui/03` AGV Status | ★★☆☆☆ (2) | Derive received cues, Inspector form | Cao |
| `webui/01` Cue Editor | ★★★★☆ (4) | React Flow drag-port, Timeline layout, đồng bộ 3 view | Trung bình — làm sau cùng |

**Khuyến nghị thứ tự:** `03 → 02 → 01 → 04` (server) rồi `02 → 03 → 01` (webui). Mỗi plan tự khai báo DoD trong mục 9; chỉ chuyển phase khi đạt.

---

## 7. HƯỚNG DẪN ĐỌC PLAN

Mỗi plan đều có 10 mục cố định theo thứ tự:
1. Bối cảnh & Mục tiêu · 2. Phạm vi · 3. Mối quan hệ & Contract · 4. Quyết định thiết kế · 5. Yêu cầu đầu ra (Deliverables) · 6. Chi tiết triển khai · 7. Độ phức tạp · 8. Quá trình thực thi · 9. Định nghĩa hoàn thành · 10. Test plan · 11. Rủi ro & Giảm thiểu · 12. Quyết định đã chốt / Câu hỏi mở.

Triển khai theo đúng thứ tự task trong mục 8, đối chiếu với mục 9 (Definition of Done) trước khi chuyển phase.
