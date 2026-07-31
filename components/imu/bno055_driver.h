#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

#include <stdint.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus

/* BNO055 register map */
#define BNO055_CHIP_ID          0xA0
#define BNO055_REG_CHIP_ID      0x00
#define BNO055_REG_PAGE_ID      0x07
#define BNO055_REG_ACC_DATA_X   0x08
#define BNO055_REG_GYR_DATA_X   0x14
#define BNO055_REG_EUL_HEADING  0x1A
#define BNO055_REG_EUL_ROLL     0x1C
#define BNO055_REG_EUL_PITCH    0x1E
#define BNO055_REG_UNIT_SEL     0x3B
#define BNO055_REG_OPR_MODE     0x3D
#define BNO055_REG_PWR_MODE     0x3E
#define BNO055_REG_SYS_TRIGGER  0x3F
#define BNO055_REG_CALIB_STAT   0x35

/* Operation modes */
#define BNO055_OPR_MODE_CONFIG  0x00
#define BNO055_OPR_MODE_NDOF    0x0C

/* Power modes */
#define BNO055_PWR_MODE_NORMAL  0x00

/* Unit selection (bit flags) */
#define BNO055_UNIT_ORIENT_DEG  0x01  /* 0=degrees, 1=radians — actually bit 0: 0=deg, 1=rad */
/* Actually: UNIT_SEL bit 0 = orientation: 0=degrees, 1=radians */
/* We want degrees (bit 0 = 0) */
#define BNO055_UNIT_SEL_VAL     0x00  /* all SI units, degrees for Euler */

class Bno055Driver {
public:
    Bno055Driver(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
                 uint32_t clk_hz = 400000, uint8_t addr = 0x28);

    /* Initialize I2C, verify chip ID, configure NDOF fusion mode */
    bool begin();

    /* Read calibrated heading in radians (±π). Fast path for EKF. */
    bool readHeadingRad(float& heading_rad);

    /* Read fused Euler angles (all in degrees) */
    bool readEuler(float& heading_deg, float& roll_deg, float& pitch_deg);

    /* Raw accelerometer (m/s²) */
    bool readAccel(float& ax, float& ay, float& az);

    /* Raw gyroscope (rad/s) */
    bool readGyro(float& gx, float& gy, float& gz);

    /* Calibration status (0-3 for sys/gyro/accel/mag) */
    void getCalStatus(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag);

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t reg, uint8_t* buf, size_t len);

    i2c_port_t port_;
    uint8_t addr_;
    gpio_num_t sda_;
    gpio_num_t scl_;
    uint32_t clk_hz_;
    bool initialized_;
    i2c_master_bus_handle_t bus_handle_;
    i2c_master_dev_handle_t dev_handle_;
};

#endif /* __cplusplus */
#endif /* BNO055_DRIVER_H */
