#include "stm_uart_link.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "StmUartLink";

StmUartLink::StmUartLink()
    : uart_num_(UART_NUM_1), cmd_rate_hz_(50), initialized_(false),
      rx_state_(WAIT_STX), rx_idx_(0), last_rx_ms_(0),
      fb_mux_(portMUX_INITIALIZER_UNLOCKED),
      safety_evt_queue_(NULL), rx_ok_count_(0), rx_crc_err_count_(0), tx_seq_(0)
{
    memset(&latest_fb_, 0, sizeof(latest_fb_));
}

void StmUartLink::begin(uart_port_t uart_num, int tx_pin, int rx_pin,
                         uint32_t baud, int cmd_rate_hz)
{
    uart_num_ = uart_num;
    cmd_rate_hz_ = cmd_rate_hz;

    uart_config_t uart_config = {};
    uart_config.baud_rate = (int)baud;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num_, 1024, 1024, 10, &safety_evt_queue_, 0));

    initialized_ = true;

    /* Start RX task */
    xTaskCreatePinnedToCore(&rxTaskFunc, "stm_rx", 2048, this, 8, NULL, 0);

    ESP_LOGI(TAG, "STM UART initialized on UART%d, baud=%lu", uart_num, (unsigned long)baud);
}

void StmUartLink::rxTaskFunc(void* arg) {
    StmUartLink* self = static_cast<StmUartLink*>(arg);
    self->rxTaskLoop();
}

void StmUartLink::processFrame() {
    uint8_t plen = rx_buf_[4];
    size_t frame_len = STM_LINK_HEADER_LEN + plen + STM_LINK_CRC_LEN;
    uint16_t crc_frame = (uint16_t)rx_buf_[frame_len - 1] << 8
                       | (uint16_t)rx_buf_[frame_len - 2];
    uint16_t crc_calc = stm_link_crc16(rx_buf_, frame_len - 2);

    if (crc_frame == crc_calc) {
        size_t off = STM_LINK_HEADER_LEN;
        uint8_t msg_id = rx_buf_[2];

        if (msg_id == MSG_MOTION_FB && plen == sizeof(motion_fb_t)) {
            motion_fb_t fb;
            memcpy(&fb, &rx_buf_[off], sizeof(fb));
            portENTER_CRITICAL(&fb_mux_);
            latest_fb_ = fb;
            last_rx_ms_ = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            portEXIT_CRITICAL(&fb_mux_);
            rx_ok_count_++;
        } else if (msg_id == MSG_SAFETY_EVENT && plen == sizeof(safety_event_t)) {
            safety_event_t evt;
            memcpy(&evt, &rx_buf_[off], sizeof(evt));
            if (safety_evt_queue_) {
                xQueueSend(safety_evt_queue_, &evt, 0);
            }
            rx_ok_count_++;
        }
    } else {
        rx_crc_err_count_++;
    }
}

void StmUartLink::rxTaskLoop() {
    uint8_t byte;

    while (1) {
        int len = uart_read_bytes(uart_num_, &byte, 1, pdMS_TO_TICKS(10));
        if (len <= 0) continue;

        switch (rx_state_) {
        case WAIT_STX:
            if (byte == STM_LINK_STX) {
                rx_buf_[0] = byte;
                rx_idx_ = 1;
                rx_state_ = WAIT_HEADER;
            }
            break;

        case WAIT_HEADER:
            rx_buf_[rx_idx_++] = byte;
            if (rx_idx_ >= STM_LINK_HEADER_LEN) {
                uint8_t plen = rx_buf_[4];
                if (plen > STM_LINK_MAX_PAYLOAD) {
                    rx_state_ = WAIT_STX;
                    break;
                }
                if (plen == 0) {
                    processFrame();
                    rx_state_ = WAIT_STX;
                    rx_idx_ = 0;
                } else {
                    rx_state_ = WAIT_PAYLOAD_CRC;
                }
            }
            break;

        case WAIT_PAYLOAD_CRC: {
            rx_buf_[rx_idx_++] = byte;
            size_t expected = STM_LINK_HEADER_LEN + rx_buf_[4] + STM_LINK_CRC_LEN;
            if (rx_idx_ >= expected) {
                processFrame();
                rx_state_ = WAIT_STX;
                rx_idx_ = 0;
            }
            break;
        }
        }
    }
}

void StmUartLink::sendMotionCmd(const motion_cmd_t& cmd) {
    if (!initialized_) return;
    uint8_t buf[STM_LINK_MAX_FRAME];
    size_t len = stm_link_encode(buf, sizeof(buf), MSG_MOTION_CMD,
                                  tx_seq_++, (const uint8_t*)&cmd, sizeof(cmd));
    if (len > 0) sendFrame(buf, len);
}

void StmUartLink::sendConfig(const config_set_t& cfg) {
    if (!initialized_) return;
    uint8_t buf[STM_LINK_MAX_FRAME];
    size_t len = stm_link_encode(buf, sizeof(buf), MSG_CONFIG_SET,
                                  tx_seq_++, (const uint8_t*)&cfg, sizeof(cfg));
    if (len > 0) sendFrame(buf, len);
}

void StmUartLink::sendFrame(const uint8_t* data, size_t len) {
    uart_write_bytes(uart_num_, data, len);
}

bool StmUartLink::getLatestFeedback(motion_fb_t& out, uint32_t max_age_ms) {
    portENTER_CRITICAL(&fb_mux_);
    out = latest_fb_;
    uint32_t rx_ms = last_rx_ms_;
    portEXIT_CRITICAL(&fb_mux_);

    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    return (now - rx_ms) < max_age_ms;
}

bool StmUartLink::linkOk() const {
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    return initialized_ && (now - last_rx_ms_) < 300;
}
