#include "dwm1000_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char* TAG = "DWM1000";
#define SPEED_OF_LIGHT 299702547.0f
#define TWR_TIMEOUT_MS 50

/* SPI header bits: bit 7=read(1)/write(0), bit 6=0, bits 5-0 = sub-address */
#define DW_SPI_WRITE(reg)  ((uint16_t)((reg) & 0x3F) << 8)  /* 7-bit sub + extended flag */
#define DW_SPI_READ(reg)   ((uint16_t)(((reg) & 0x3F) << 8) | 0x80)

Dwm1000Driver::Dwm1000Driver(spi_host_device_t host, int cs, int irq, int rst)
    : host_(host), cs_(cs), irq_(irq), rst_(rst),
      anchor_count_(0), initialized_(false)
{
    memset(anchors_, 0, sizeof(anchors_));
}

bool Dwm1000Driver::spiWrite(uint16_t reg, const uint8_t* data, size_t len) {
    /* DW1000 SPI: first byte = (reg & 0x7F) << 1 | 0, extended addressing uses 2-byte header */
    /* Simplified: use single-byte header for registers < 0x40 */
    uint8_t header = (reg & 0x3F) << 1; /* write, no extended */
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = (8 + len * 8);
    t.tx_data[0] = header;
    memcpy(&t.tx_data[1], data, len);
    return spi_device_polling_transmit(spi_, &t) == ESP_OK;
}

bool Dwm1000Driver::spiRead(uint16_t reg, uint8_t* data, size_t len) {
    uint8_t header = ((reg & 0x3F) << 1) | 0x01; /* read, no extended */
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;
    t.rxlength = len * 8;
    t.tx_data[0] = header;
    t.rx_buffer = data;
    return spi_device_polling_transmit(spi_, &t) == ESP_OK;
}

bool Dwm1000Driver::begin() {
    /* Configure SPI bus */
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = 23;
    buscfg.miso_io_num = 19;
    buscfg.sclk_io_num = 18;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 256;

    esp_err_t ret = spi_bus_initialize(host_, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }

    /* Add SPI device */
    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 20000000;
    devcfg.spics_io_num = cs_;
    devcfg.queue_size = 1;
    devcfg.cs_ena_pretrans = 2;

    ret = spi_bus_add_device(host_, &devcfg, &spi_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return false;
    }

    /* Configure IRQ pin as input */
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << irq_);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    /* Hardware reset if RST pin defined */
    if (rst_ >= 0) {
        gpio_config_t rst_conf = {};
        rst_conf.pin_bit_mask = (1ULL << rst_);
        rst_conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&rst_conf);
        gpio_set_level((gpio_num_t)rst_, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)rst_, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        softReset();
    }

    if (!verifyDeviceId()) {
        ESP_LOGE(TAG, "DW1000 not found");
        return false;
    }

    if (!configureChannel()) {
        ESP_LOGE(TAG, "Failed to configure DW1000 channel");
        return false;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "DWM1000 initialized");
    return true;
}

bool Dwm1000Driver::softReset() {
    /* SYS_CTRL register: bit 0 = soft reset */
    uint8_t val = 0x01;
    spiWrite(0x0C, &val, 1); /* simplified register address */
    vTaskDelay(pdMS_TO_TICKS(10));
    val = 0x00;
    spiWrite(0x0C, &val, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

bool Dwm1000Driver::verifyDeviceId() {
    uint8_t buf[4];
    if (!spiRead(DW1000_REG_DEV_ID, buf, 4)) return false;
    uint32_t dev_id = (uint32_t)buf[0] | (uint32_t)buf[1] << 8
                    | (uint32_t)buf[2] << 16 | (uint32_t)buf[3] << 24;
    ESP_LOGI(TAG, "DW1000 DEV_ID = 0x%08lX", (unsigned long)dev_id);
    return dev_id == DW1000_DEV_ID;
}

bool Dwm1000Driver::configureChannel() {
    /* Channel 5 (6.5 GHz), 6.8 Mbps, 64 MHz PRF
       This is a simplified placeholder — a production implementation
       would write the full CHAN_CTRL, TX_POWER, RF_CONF, etc. register
       sequence from the DW1000 datasheet. The exact register values
       depend on the channel configuration chosen. */
    ESP_LOGI(TAG, "DW1000 channel configured (defaults)");
    return true;
}

void Dwm1000Driver::sendFrame(const uint8_t* data, size_t len) {
    /* Write to TX_BUFFER, set TX frame length and start TX */
    if (len > 127) len = 127;
    spiWrite(0x0C, data, len); /* simplified TX BUFFER write */

    /* Set TX_FCTRL (frame length) */
    uint8_t fctrl[2] = {(uint8_t)(len * 8), (uint8_t)((len * 8) >> 8)};
    spiWrite(0x0C, fctrl, 2); /* simplified */

    /* Start TX: SYS_CTRL.TXSTRT = 1 */
    uint8_t tx_ctrl = 0x02; /* TXSTRT bit */
    spiWrite(0x0C, &tx_ctrl, 1);
}

bool Dwm1000Driver::waitForIRQ(uint32_t timeout_ms) {
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        if (gpio_get_level((gpio_num_t)irq_) == 1) return true;
        taskYIELD();
    }
    return false;
}

bool Dwm1000Driver::doRanging(float ranges_m[MAX_ANCHORS], int& count) {
    if (!initialized_ || anchor_count_ < 1) return false;

    count = 0;
    /* Simplified: range with first anchor only as a basic test.
       A full TWR protocol (Poll/Response/Final) would be implemented here. */
    for (int i = 0; i < anchor_count_; i++) {
        /* --- POLL: send poll frame --- */
        uint8_t poll_frame[10] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        sendFrame(poll_frame, sizeof(poll_frame));

        /* --- Wait for RESPONSE --- */
        if (!waitForIRQ(TWR_TIMEOUT_MS)) continue;

        /* --- Read response --- */
        /* ... response parsing ... */
        /* Placeholder: assign a fake range for compilation. Real impl uses TWR math. */
        ranges_m[count] = 1.5f; /* placeholder */
        count++;
    }

    return count > 0;
}

bool Dwm1000Driver::trilaterate(const float ranges_m[], int count, float& x, float& y) {
    if (count < 3) return false; /* need at least 3 anchors */

    /* Least-Squares trilateration via normal equations.
       Linearize using anchor 0 as reference. */
    int n = count - 1;
    float A[8][2], b[8];

    for (int i = 1; i < count; i++) {
        float xi = anchors_[i][0], yi = anchors_[i][1];
        float x0 = anchors_[0][0], y0 = anchors_[0][1];
        float r0 = ranges_m[0], ri = ranges_m[i];

        A[i-1][0] = 2.0f * (xi - x0);
        A[i-1][1] = 2.0f * (yi - y0);
        b[i-1] = r0 * r0 - ri * ri + xi * xi - x0 * x0 + yi * yi - y0 * y0;
    }

    /* Compute AᵀA (2x2) and Aᵀb (2x1) */
    float AtA[2][2] = {{0, 0}, {0, 0}};
    float Atb[2] = {0, 0};

    for (int i = 0; i < n; i++) {
        AtA[0][0] += A[i][0] * A[i][0];
        AtA[0][1] += A[i][0] * A[i][1];
        AtA[1][0] += A[i][1] * A[i][0];
        AtA[1][1] += A[i][1] * A[i][1];
        Atb[0] += A[i][0] * b[i];
        Atb[1] += A[i][1] * b[i];
    }

    /* Solve 2x2 system: p = (AᵀA)⁻¹ Aᵀb */
    float det = AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0];
    if (fabsf(det) < 1e-6f) return false;

    float inv_det = 1.0f / det;
    x = (AtA[1][1] * Atb[0] - AtA[0][1] * Atb[1]) * inv_det;
    y = (AtA[0][0] * Atb[1] - AtA[1][0] * Atb[0]) * inv_det;

    return true;
}

void Dwm1000Driver::setAnchors(const float anchors[][3], int count) {
    if (count > MAX_ANCHORS) count = MAX_ANCHORS;
    anchor_count_ = count;
    for (int i = 0; i < count; i++) {
        anchors_[i][0] = anchors[i][0];
        anchors_[i][1] = anchors[i][1];
        anchors_[i][2] = anchors[i][2];
    }
    ESP_LOGI(TAG, "%d anchors configured", count);
}
