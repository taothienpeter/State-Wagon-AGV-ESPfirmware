#include "bno055_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char* TAG = "BNO055";
#define DEG2RAD 0.01745329252f

Bno055Driver::Bno055Driver(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
                           uint32_t clk_hz, uint8_t addr)
    : port_(port), addr_(addr), sda_(sda), scl_(scl), clk_hz_(clk_hz), initialized_(false)
{
}

bool Bno055Driver::writeReg(uint8_t reg, uint8_t val) {
    esp_err_t ret;
    uint8_t buf[2] = {reg, val};

    /* Use i2c_master_transmit for the ESP-IDF I2C master driver */
    ret = i2c_master_transmit(dev_handle_, buf, 2, -1);
    return ret == ESP_OK;
}

bool Bno055Driver::readRegs(uint8_t reg, uint8_t* buf, size_t len) {
    esp_err_t ret;
    ret = i2c_master_transmit_receive(dev_handle_, &reg, 1, buf, len, -1);
    return ret == ESP_OK;
}

bool Bno055Driver::begin() {
    /* Add I2C bus and device */
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = port_;
    bus_config.sda_io_num = sda_;
    bus_config.scl_io_num = scl_;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return false;
    }

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = addr_;
    dev_config.scl_speed_hz = clk_hz_;

    ret = i2c_master_bus_add_device(bus_handle_, &dev_config, &dev_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return false;
    }

    /* Verify chip ID */
    uint8_t chip_id = 0;
    if (!readRegs(BNO055_REG_CHIP_ID, &chip_id, 1)) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return false;
    }

    if (chip_id != BNO055_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected BNO055 chip ID: 0x%02X (expected 0x%02X)",
                 chip_id, BNO055_CHIP_ID);
        return false;
    }
    ESP_LOGI(TAG, "BNO055 found, chip ID = 0x%02X", chip_id);

    /* Switch to CONFIG mode */
    writeReg(BNO055_REG_OPR_MODE, BNO055_OPR_MODE_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Set units: degrees for Euler angles */
    writeReg(BNO055_REG_UNIT_SEL, BNO055_UNIT_SEL_VAL);

    /* Set power mode to NORMAL */
    writeReg(BNO055_REG_PWR_MODE, BNO055_PWR_MODE_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Switch to NDOF fusion mode */
    writeReg(BNO055_REG_OPR_MODE, BNO055_OPR_MODE_NDOF);
    vTaskDelay(pdMS_TO_TICKS(20));

    initialized_ = true;
    ESP_LOGI(TAG, "BNO055 initialized in NDOF mode");
    return true;
}

bool Bno055Driver::readHeadingRad(float& heading_rad) {
    if (!initialized_) return false;

    uint8_t buf[2];
    if (!readRegs(BNO055_REG_EUL_HEADING, buf, 2)) {
        return false;
    }

    /* Little-endian int16, 1° = 16 LSB */
    int16_t raw = (int16_t)(buf[0] | (buf[1] << 8));
    float heading_deg = (float)raw / 16.0f;

    /* Normalize to ±π */
    heading_rad = heading_deg * DEG2RAD;
    heading_rad = atan2f(sinf(heading_rad), cosf(heading_rad));
    return true;
}

bool Bno055Driver::readEuler(float& heading_deg, float& roll_deg, float& pitch_deg) {
    if (!initialized_) return false;

    uint8_t buf[6]; /* H(2) + R(2) + P(2) */
    if (!readRegs(BNO055_REG_EUL_HEADING, buf, 6)) {
        return false;
    }

    int16_t h_raw = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t r_raw = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t p_raw = (int16_t)(buf[4] | (buf[5] << 8));

    heading_deg = (float)h_raw / 16.0f;
    roll_deg    = (float)r_raw / 16.0f;
    pitch_deg   = (float)p_raw / 16.0f;
    return true;
}

bool Bno055Driver::readAccel(float& ax, float& ay, float& az) {
    if (!initialized_) return false;

    uint8_t buf[6];
    if (!readRegs(BNO055_REG_ACC_DATA_X, buf, 6)) {
        return false;
    }

    int16_t x_raw = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t y_raw = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t z_raw = (int16_t)(buf[4] | (buf[5] << 8));

    /* Default unit: m/s², 1 LSB = 0.01 m/s² */
    ax = (float)x_raw * 0.01f;
    ay = (float)y_raw * 0.01f;
    az = (float)z_raw * 0.01f;
    return true;
}

bool Bno055Driver::readGyro(float& gx, float& gy, float& gz) {
    if (!initialized_) return false;

    uint8_t buf[6];
    if (!readRegs(BNO055_REG_GYR_DATA_X, buf, 6)) {
        return false;
    }

    int16_t x_raw = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t y_raw = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t z_raw = (int16_t)(buf[4] | (buf[5] << 8));

    /* Default unit: deg/s, 1 LSB = 1/16 deg/s */
    gx = (float)x_raw / 16.0f * DEG2RAD;
    gy = (float)y_raw / 16.0f * DEG2RAD;
    gz = (float)z_raw / 16.0f * DEG2RAD;
    return true;
}

void Bno055Driver::getCalStatus(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag) {
    uint8_t status = 0;
    if (readRegs(BNO055_REG_CALIB_STAT, &status, 1)) {
        sys   = (status >> 6) & 0x03;
        gyro  = (status >> 4) & 0x03;
        accel = (status >> 2) & 0x03;
        mag   = status & 0x03;
    } else {
        sys = gyro = accel = mag = 0;
    }
}
