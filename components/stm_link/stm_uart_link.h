#ifndef STM_UART_LINK_H
#define STM_UART_LINK_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"

#include "stm_link.h"

#ifdef __cplusplus

class StmUartLink {
public:
    StmUartLink();

    /* Install UART driver, start RX task. Call once after FreeRTOS starts. */
    void begin(uart_port_t uart_num, int tx_pin, int rx_pin,
               uint32_t baud, int cmd_rate_hz);

    /* Send a motion command frame */
    void sendMotionCmd(const motion_cmd_t& cmd);

    /* Send config set frame (rare, e.g. at startup) */
    void sendConfig(const config_set_t& cfg);

    /* Thread-safe read of latest feedback. Returns false if too old. */
    bool getLatestFeedback(motion_fb_t& out, uint32_t max_age_ms = 200);

    /* Safety event queue */
    bool hasPendingSafetyEvent() { return safety_evt_queue_ != NULL && uxQueueMessagesWaiting(safety_evt_queue_) > 0; }
    bool popSafetyEvent(safety_event_t& evt) {
        return safety_evt_queue_ && xQueueReceive(safety_evt_queue_, &evt, 0) == pdTRUE;
    }

    /* Diagnostics */
    bool linkOk() const;
    uint32_t framesReceivedOk() const { return rx_ok_count_; }
    uint32_t framesCrcError() const { return rx_crc_err_count_; }

private:
    static void rxTaskFunc(void* arg);
    void rxTaskLoop();

    /* Send raw frame (with lock) */
    void sendFrame(const uint8_t* data, size_t len);

    /* Process CRC and dispatch a complete frame in rx_buf_ */
    void processFrame();

    uart_port_t uart_num_;
    int cmd_rate_hz_;
    bool initialized_;

    /* RX state machine */
    enum RxState { WAIT_STX, WAIT_HEADER, WAIT_PAYLOAD_CRC };
    RxState rx_state_;
    uint8_t rx_buf_[STM_LINK_MAX_FRAME];
    size_t rx_idx_;

    /* Latest feedback (protected by spinlock) */
    motion_fb_t latest_fb_;
    uint32_t last_rx_ms_;
    portMUX_TYPE fb_mux_;

    /* Safety event queue */
    QueueHandle_t safety_evt_queue_;

    /* Counters */
    uint32_t rx_ok_count_;
    uint32_t rx_crc_err_count_;

    /* Sequence counter for TX */
    uint8_t tx_seq_;
};

#endif /* __cplusplus */
#endif /* STM_UART_LINK_H */
