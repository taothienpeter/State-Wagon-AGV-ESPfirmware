#ifndef STM_LINK_H
#define STM_LINK_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Frame constants ---- */
#define STM_LINK_STX             0xAA
#define STM_LINK_VERSION         0x01
#define STM_LINK_HEADER_LEN      5     /* STX(1) + VERSION(1) + MSG_ID(1) + SEQ(1) + LEN(1) */
#define STM_LINK_CRC_LEN         2
#define STM_LINK_MAX_PAYLOAD     64
#define STM_LINK_MAX_FRAME       (STM_LINK_HEADER_LEN + STM_LINK_MAX_PAYLOAD + STM_LINK_CRC_LEN)

/* ---- Message IDs ---- */
#define MSG_MOTION_CMD      0x01
#define MSG_MOTION_FB       0x02
#define MSG_SAFETY_EVENT    0x03
#define MSG_CONFIG_SET      0x04

/* ---- Mode values ---- */
#define MODE_IDLE           0x00
#define MODE_POSITION       0x01
#define MODE_SAFE_STOP      0x02

/* ---- Safety states ---- */
#define SAFETY_NORMAL       0x00
#define SAFETY_WARN         0x01
#define SAFE_STOP           0x02
#define FAULT_LATCHED       0x03

/* ---- Safety event sources ---- */
#define SRC_E_STOP          0x00
#define SRC_DRIVE_FAULT     0x01
#define SRC_STEPPER_STALL   0x02
#define SRC_HOMING_FAILED   0x03
#define SRC_CMD_TIMEOUT     0x04

/* ---- Structs ---- */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;               // 4
    float    drive_pos_target_turns;     // 4, accumulated position in turns
    float    steering_angle_rad;         // 4, range: -π/2 to +π/2
    float    steering_velocity_radps;    // 4, rate limit hint for stepper
    uint8_t  mode;                       // 1
    uint8_t  flags;                      // 1
    uint8_t  _pad[2];                    // 2, reserved
} motion_cmd_t;  /* 20 bytes */

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
} motion_fb_t;  /* 29 bytes */

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  source;
    uint8_t  new_state;
} safety_event_t;  /* 6 bytes */

typedef struct __attribute__((packed)) {
    float    drive_wheel_radius_m;
    float    wheelbase_m;
    float    max_drive_velocity_mps;
    float    max_steering_angle_rad;
} config_set_t;  /* 16 bytes */

/* ---- Compile-time struct size verification ---- */
_Static_assert(sizeof(motion_cmd_t) == 20, "motion_cmd_t must be 20 bytes");
_Static_assert(sizeof(motion_fb_t) == 29, "motion_fb_t must be 29 bytes");
_Static_assert(sizeof(safety_event_t) == 6, "safety_event_t must be 6 bytes");
_Static_assert(sizeof(config_set_t) == 16, "config_set_t must be 16 bytes");

/* ---- CRC16-CCITT ---- */
static inline uint16_t stm_link_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ---- Encode helper: writes a complete frame into buf, returns total bytes written ---- */
static inline size_t stm_link_encode(uint8_t *buf, size_t cap,
                                     uint8_t msg_id, uint8_t seq,
                                     const uint8_t *payload, uint8_t len)
{
    if (cap < (size_t)STM_LINK_HEADER_LEN + len + STM_LINK_CRC_LEN)
        return 0;
    size_t idx = 0;
    buf[idx++] = STM_LINK_STX;
    buf[idx++] = STM_LINK_VERSION;
    buf[idx++] = msg_id;
    buf[idx++] = seq;
    buf[idx++] = len;
    if (payload && len > 0) {
        memcpy(&buf[idx], payload, len);
        idx += len;
    }
    uint16_t crc = stm_link_crc16(buf, idx);
    buf[idx++] = crc & 0xFF;
    buf[idx++] = (crc >> 8) & 0xFF;
    return idx;
}

/* ---- Decode return values ---- */
#define STM_DECODE_OK           0
#define STM_DECODE_NEED_MORE    1
#define STM_DECODE_BAD_STX      2
#define STM_DECODE_CRC_ERR      3
#define STM_DECODE_TOO_SHORT    4

/* ---- Decode result ---- */
typedef struct {
    uint8_t  msg_id;
    uint8_t  seq;
    uint8_t  len;
    uint8_t  payload[STM_LINK_MAX_PAYLOAD];
} stm_link_frame_t;

/* ---- Decode: feed bytes; returns status + fills out on complete frame ---- */
/*     Returns STM_DECODE_OK only when a complete valid frame is found.     */
/*     offset is updated to the byte after the decoded frame, so you can    */
/*     call repeatedly on a buffer with possible trailing data.             */
static inline int stm_link_decode(const uint8_t *buf, size_t buf_len,
                                  size_t *offset, stm_link_frame_t *out)
{
    size_t off = *offset;
    while (off + STM_LINK_HEADER_LEN <= buf_len) {
        if (buf[off] != STM_LINK_STX) {
            off++;
            continue;
        }
        uint8_t ver   = buf[off + 1];
        (void)ver;
        uint8_t id    = buf[off + 2];
        uint8_t seq   = buf[off + 3];
        uint8_t plen  = buf[off + 4];
        size_t frame_len = STM_LINK_HEADER_LEN + plen + STM_LINK_CRC_LEN;
        if (plen > STM_LINK_MAX_PAYLOAD || frame_len > STM_LINK_MAX_FRAME) {
            off++;
            continue;
        }
        if (off + frame_len > buf_len) {
            *offset = off;
            return STM_DECODE_NEED_MORE;
        }
        uint16_t crc_frame = (uint16_t)buf[off + frame_len - 1] << 8
                           | (uint16_t)buf[off + frame_len - 2];
        uint16_t crc_calc = stm_link_crc16(&buf[off], frame_len - 2);
        if (crc_frame != crc_calc) {
            off++;
            continue;
        }
        out->msg_id = id;
        out->seq    = seq;
        out->len    = plen;
        if (plen > 0)
            memcpy(out->payload, &buf[off + STM_LINK_HEADER_LEN], plen);
        *offset = off + frame_len;
        return STM_DECODE_OK;
    }
    *offset = off;
    return (off + STM_LINK_HEADER_LEN > buf_len) ? STM_DECODE_NEED_MORE : STM_DECODE_TOO_SHORT;
}

#ifdef __cplusplus
}
#endif

#endif /* STM_LINK_H */
