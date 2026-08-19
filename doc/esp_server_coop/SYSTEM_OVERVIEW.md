# StageWagonServer — Giới thiệu hệ thống Server ↔ ESP32

> File giới thiệu tổng quan về **kiến trúc server** và **tech stack** của hệ thống điều
> khiển đội xe AGV "Stage Wagon" trên sân khấu. Cập nhật **2026-08-07**.
> Chi tiết wire protocol xem `Plans/MESSAGE_PROTOCOL.md`; kế hoạch/quyết định xem
> `Plans/StageWagonServer/` (vault Obsidian).

---

## 1. Hệ thống này là gì?

**StageWagonServer** là máy chủ điều khiển một đội xe AGV tự hành ("Stage Wagon") di
chuyển cùng con người (diễn viên) trên sân khấu theo các **bài biểu diễn (show)** được
soạn trước. Mỗi show là một chuỗi **cue** (mệnh lệnh), mỗi cue gồm các **move**
(một hoặc nhiều AGV di chuyển tới đích với tốc độ/tolerance/bo góc riêng).

Hai lớp chính:
- **Server (StageWagonServer)** — "bộ não": quản lý show, điều phối cue, lập lộ trình
  di chuyển, giám sát trạng thái AGV, cung cấp web UI.
- **ESP32 (firmware)** — "cánh tay": kết nối TCP tới server, nhận lệnh `order`,
  gửi lại `state` (~8Hz) + `ack`, thực thi chuyển động. **Giai đoạn hiện tại dùng
  simulator `tools/fake_esp32.py`**; firmware thật (Phase 3) chưa tích hợp.

Giao tiếp Server ↔ ESP32: **TCP + JSON, length-prefixed** (`[u32BE length][UTF-8 JSON]`),
đơn giản hóa theo nhu cầu sân khấu — **không** dùng VDA 5050 / MQTT.

---

## 2. Kiến trúc tổng thể

```
                 ┌─────────────────────────────────────────────┐
                 │            WEB UI (React/Vite)               │
                 │   Cue Editor · Live Monitor · AGV Status     │
                 └──────────▲──────────────────▲───────────────┘
                      REST /api/*         WS /api/ws (snapshot 0.25s)
                 ┌──────────┴──────────────────┴───────────────┐
                 │                app/api/                      │
                 │         routes.py (REST) + ws.py (WS)        │
                 └──────────▲──────────────────▲───────────────┘
                 ┌──────────┴──────────────────┴───────────────┐
                 │            app/show.py  (ShowManager)        │
                 │         state machine 2 tầng + supervisor    │
                 └───────▲───────────────┬─────────────▲───────┘
                 ┌───────┴──────┐  ┌─────┴──────┐  ┌───┴────────┐
                 │ app/fleet.py │  │ app/estop  │  │ app/motion │
                 │ FleetService │  │ E-STOP     │  │ MPG (MP)   │
                 └───────▲──────┘  └────────────┘  └───▲────────┘
                 ┌───────┴──────────────┐  ┌──────────┴─────────┐
                 │  app/tcp_server.py   │  │  app/registry.py    │
                 │  TCP :7800 (ESP32)   │  │  VehicleRegistry    │
                 └───────────▲──────────┘  └─────────────────────┘
                             │ TCP :7800, [len][JSON]
                 ┌───────────┴───────────┐
                 │ ESP32 (Phase 3)        │  ← hiện tại: tools/fake_esp32.py
                 └───────────────────────┘
```

**Luồng dữ liệu chính:**
1. ESP32/simulator kết nối TCP tới `:7800`, gửi `connection` (ONLINE + capabilities) rồi `state` (~8Hz).
2. `tcp_server.py` → `registry.py` cập nhật vị trí/trạng thái/orderState/ack.
3. Người vận hành bấm **GO** (Web UI) → REST → `show.py` → `fleet.py` → `order` qua TCP tới từng AGV.
4. `show.py` supervisor **poll position** để xác định cue hoàn thành → auto-advance sang cue kế.
5. `ws.py` broadcast snapshot 0.25s tới Web UI (agvs + runtime + estop + stage).

---

## 3. Thành phần server (mô-đun) và trách nhiệm

| Mô-đun | Trách nhiệm |
|---|---|
| `app/main.py` | Entrypoint, wiring services, broadcast loop, mount `/studio` (web build). |
| `app/protocol.py` | **Wire contract**: framing `[u32BE len][JSON]`, envelope `{type, vehicleId, payload}`, validate + `make_order/make_instant_action/make_ack`, giới hạn 1 MiB. |
| `app/tcp_server.py` | TCP server đa AGV (`:7800`), 1 connection persistent/AGV, xử lý message → registry, an toàn khi reconnect. |
| `app/registry.py` | `VehicleRegistry`: trạng thái live AGV, OFFLINE timeout (3s) + sweep, `recent_orders` (≤10), theo dõi `orderId`/`ack_status`/`cueId`, `snapshot()`. |
| `app/motion.py` | `MotionPlanner` (MPG): validate tọa độ trong sân khấu, clamp tốc độ theo capabilities, bo góc (`corner_blend_m`), giới hạn ≤200 waypoint. |
| `app/fleet.py` | `FleetService`: gửi `order` (confirmed dispatch + ack polling), `instantAction`, broadcast E-STOP, log `orders.jsonl`. |
| `app/show.py` | `ShowManager`: **state machine 2 tầng** + supervisor asyncio/cue, blocking/timeout/dwell/auto-advance, **parallel cue**, recovery (clear_fault/clear_estop). |
| `app/estop.py` | `EStopHandler`: E-STOP toàn cục (gửi ngay tới mọi xe). |
| `app/storage.py` | Lưu file: show JSON (atomic), agvs JSON, orders/state JSONL, chống path traversal. |
| `app/config.py` | Cấu hình `config.json` + env override (host/port/offline/stage/ack...). |
| `app/api/routes.py` | REST API (health, agvs, estop, order, shows CRUD, go/pause/resume/skip/clear-fault, runtime, stage). |
| `app/api/ws.py` | WebSocket `/api/ws` — broadcast snapshot realtime 0.25s. |

### State machine 2 tầng (`app/show.py`)
- **Show-level:** `STOPPED / RUNNING / PAUSED / FAULT / ESTOP / COMPLETED`.
- **Cue-level:** `PENDING / READY / RUNNING / COMPLETED / TIMEOUT / ERROR / SKIPPED / ABORTED`.
- Mỗi cue chạy bởi một **supervisor asyncio** (dispatch → poll hoàn thành → timeout/dwell → auto-advance). Mọi transition bọc `asyncio.Lock`.
- Hoàn thành cue: **poll position** trong `tolerance_m` là nguồn chính; `orderState=COMPLETED` chỉ là fallback.

---

## 4. Tech Stack

### Backend
| Công nghệ | Vai trò |
|---|---|
| **Python 3.13** | Ngôn ngữ server. |
| **FastAPI** | Framework REST API + WebSocket (docs tại `/docs`). |
| **Uvicorn** (asyncio) | ASGI server, chạy HTTP `:8000`. |
| **asyncio** | TCP server, supervisor cue, broadcast — toàn bộ I/O bất đồng bộ đơn luồng. |
| **Pydantic v2** | Model validate cho REST (`ShowIn/CueIn/MoveIn/SceneIn`). |
| **pytest + pytest-asyncio + httpx** | Unit + integration test (87 test). |

### Frontend (Web UI, project `web/`)
| Công nghệ | Vai trò |
|---|---|
| **Vite** | Dev server + build tool. |
| **React 18 + TypeScript** | Giao diện. |
| **Zustand** | Shared store (snapshot AGV/runtime/estop + selection + currentShow). |
| **@xyflow/react (React Flow)** | Flow Graph soạn cue. |
| **@dagrejs/dagre** | Auto-layout node graph theo scene. |
| **Vitest** | Unit test lib (45 test). |

### Simulator & CLI (tools/)
| File | Vai trò |
|---|---|
| `tools/fake_esp32.py` | Giả lập ESP32: kết nối TCP, gửi `state` ~8Hz + `ack` + `orderState`, di chuyển theo waypoint — phát triển không cần phần cứng. |
| `tools/cli.py` | Lệnh nhanh từ terminal (agvs, estop, order, shows, create-show). |

### Lưu trữ & Cấu hình
- `data/shows/*.json` — nội dung show (schema v2: scenes/cues/moves).
- `data/agvs.json`, `data/orders.jsonl`, `data/state_log.jsonl` — trạng thái + log.
- `config.json` + env override (`HOST`, `PORT`, `HTTP_HOST`, `HTTP_PORT`, `OFFLINE_TIMEOUT_S`...).

---

## 5. Giao thức Server ↔ ESP32 (tóm tắt)

Framing `[u32BE length][UTF-8 JSON]`, envelope `{type, vehicleId, payload}`. TCP `:7800`.

| Message | Chiều | Payload |
|---|---|---|
| `order` | Server → ESP32 | `{orderId, waypoints:[{x,y,max_speed_mps,tolerance_m?}]}` |
| `instantAction` | Server → ESP32 | `{actionType: emergencyStop\|startPause\|stopPause\|clearFault}` |
| `state` | ESP32 → Server (~8Hz) | `{orderId, orderState, operatingMode, safetyState, position, velocity, battery, imu, uwb, driveControllers[]}` |
| `connection` | ESP32 → Server | `{connectionState: ONLINE, capabilities?}` |
| `ack` | ESP32 → Server | `{orderId, status: ACCEPTED\|REJECTED, reason?}` |

- Xác nhận đơn hàng tùy chọn qua cờ `order_ack_enabled` (mặc định **false** vì firmware chưa gửi ack); khi bật, server poll ack trong `order_ack_timeout_s` (2s).
- OFFLINE khi không nhận `state` trong `offline_timeout_s` (3s).

Chi tiết đầy đủ: `Plans/MESSAGE_PROTOCOL.md`.

---

## 6. Web UI (3 tab)

- **Cue Editor** — soạn show: Flow Graph (kéo thả nối cue theo `after`), SceneManager
  (phân cấp show → scene → cue, `parallelPolicy`), CueTable (bảng, reorder), Timeline
  (kéo clip đặt `startMs`/`durationMs`), Inspector (General/Moves/Dependencies/Advanced).
- **Live Monitor** — vận hành: StageCanvas (vẽ AGV, tools select/place/measure/pan/drag),
  transport (GO/Pause/Resume/Skip/ClearFault/E-STOP), timeline CTP/RTP, thông tin đối tượng.
- **AGV Status** — giám sát: card từng xe (vị trí, pin, mode, safety, cue đã nhận), AGV
  Inspector (điều khiển thủ công từng xe).

Dev: Vite `:5173` (proxy `/api`→`:8000`). Production: `npm run build` → FastAPI mount `web/dist` tại `/studio`.

---

## 7. Cách chạy

```bash
# backend
python -m venv .venv && .venv\Scripts\activate
pip install -r requirements.txt
python -m app.main                 # TCP :7800 + HTTP :8000

# simulator (không cần phần cứng)
python tools/fake_esp32.py --count 3

# web UI (dev)
cd web && npm install && npm run dev    # mở http://localhost:5173

# test
python -m pytest -q                       # 87 backend
cd web && npm run test && npm run typecheck   # 45 frontend
```

---

## 8. Trạng thái & giai đoạn

- **Phase 1 (server)** ✅ — toàn bộ module backend + 87 test pass.
- **Phase 2 (web UI)** ✅ — 3 tab + 45 test pass + typecheck sạch.
- **Phase 3 (firmware ESP32 thật)** ⬜ — **chưa bắt đầu**; hiện dùng simulator.
- Hệ thống chạy end-to-end với `fake_esp32.py`.

Trạng thái chi tiết, quyết định thiết kế và lộ trình: thư mục vault
`Plans/StageWagonServer/` (`STATUS.md`, `decisions.md`, `progress.md`, `server/`, `webui/`).
