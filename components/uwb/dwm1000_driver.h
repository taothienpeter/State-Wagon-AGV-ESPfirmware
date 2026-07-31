#ifndef DWM1000_DRIVER_H
#define DWM1000_DRIVER_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus

#define MAX_ANCHORS 8

/* DW1000 device ID (reg 0x00) */
#define DW1000_DEV_ID      0xDECA0130

/* Register addresses (shifted for SPI — 7-bit sub-address, DW1000 auto-increments) */
#define DW1000_REG_DEV_ID  0x00
#define DW1000_REG_SYS_CFG 0x04
#define DW1000_REG_SYS_CTRL 0x0C
#define DW1000_REG_SYS_STATUS 0x08
#define DW1000_REG_TX_FCTRL 0x0C
#define DW1000_REG_TX_BUFFER 0x0C  /* 0C with sub-address */

class Dwm1000Driver {
public:
    Dwm1000Driver(spi_host_device_t host, int cs, int irq, int rst = -1);

    /* Init SPI bus, configure DW1000 channel */
    bool begin();

    /* Blocking TWR cycle: range with all known anchors */
    bool doRanging(float ranges_m[MAX_ANCHORS], int& count);

    /* Least-Squares trilateration: ranges → (x, y) */
    bool trilaterate(const float ranges_m[], int count, float& x, float& y);

    /* Configure known anchor positions (x, y, z in meters) */
    void setAnchors(const float anchors[][3], int count);

private:
    /* Low-level SPI register access */
    bool spiRead(uint16_t reg, uint8_t* data, size_t len);
    bool spiWrite(uint16_t reg, const uint8_t* data, size_t len);

    bool softReset();
    bool verifyDeviceId();
    bool configureChannel();
    void sendFrame(const uint8_t* data, size_t len);
    bool waitForIRQ(uint32_t timeout_ms);

    spi_host_device_t host_;
    spi_device_handle_t spi_;
    int cs_, irq_, rst_;
    float anchors_[MAX_ANCHORS][3];
    int anchor_count_;
    bool initialized_;
};

#endif /* __cplusplus */
#endif /* DWM1000_DRIVER_H */
