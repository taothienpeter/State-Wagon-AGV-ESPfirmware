#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Identity & Network ---- */
#define AGV_DEFAULT_VEHICLE_ID      "agv-test-01"
#define AGV_DEFAULT_WIFI_SSID       "Wifi_Nha_Ban"
#define AGV_DEFAULT_WIFI_PASS       "Mat_Khau_Wifi"
#define AGV_DEFAULT_SERVER_HOST     "192.168.1.100"
#define AGV_DEFAULT_SERVER_PORT     7800

/* ---- Timing ---- */
#define AGV_HEARTBEAT_PERIOD_MS     1000
#define AGV_RECONNECT_BACKOFF_MIN_MS 500
#define AGV_RECONNECT_BACKOFF_MAX_MS 15000
#define AGV_STATE_PUBLISH_HZ        8
#define AGV_WIFI_CONNECT_TIMEOUT_MS 15000

/* ---- UART (to STM32) ---- */
#define STM_UART_NUM                UART_NUM_1
#define STM_TX_PIN                  GPIO_NUM_17
#define STM_RX_PIN                  GPIO_NUM_16
#define STM_UART_BAUD               921600
#define STM_CMD_RATE_HZ             50
#define STM_FB_TIMEOUT_MS           300

/* ---- Sensor Enable Flags ---- */
#define AGV_ENABLE_IMU              1   /* 1 = BNO055 active (Yaw/Heading source) */
#define AGV_ENABLE_UWB              0   /* 0 = UWB disabled/bypassed */

/* ---- IMU (BNO055 via I2C) ---- */
#define IMU_I2C_NUM                 I2C_NUM_0
#define IMU_I2C_SDA_PIN             GPIO_NUM_21
#define IMU_I2C_SCL_PIN             GPIO_NUM_22
#define IMU_I2C_CLOCK_HZ            400000
#define IMU_I2C_ADDR                0x28
#define IMU_SAMPLE_RATE_HZ          100

/* ---- UWB (DWM1000 via SPI - disabled when AGV_ENABLE_UWB=0) ---- */
#define UWB_SPI_HOST                SPI2_HOST
#define UWB_SPI_MOSI                GPIO_NUM_23
#define UWB_SPI_MISO                GPIO_NUM_19
#define UWB_SPI_CLK                 GPIO_NUM_18
#define UWB_SPI_CS                  GPIO_NUM_5
#define UWB_SPI_IRQ                 GPIO_NUM_4
#define UWB_SPI_CLOCK_HZ           20000000
#define UWB_RATE_HZ                 10
#define UWB_ANCHOR_COUNT            0

/* ---- Vehicle geometry ---- */
#define AGV_WHEELBASE_M             0.42f
#define AGV_WHEEL_RADIUS_M          0.075f
#define AGV_MAX_VELOCITY_MPS        1.2f
#define AGV_MAX_STEERING_ANGLE_RAD  1.57f
#define AGV_MAX_STEERING_RATE_RADPS 3.14f

/* ---- Motion profile ---- */
#define AGV_MAX_ACCEL_MPS2          0.5f
#define AGV_MAX_DECEL_MPS2          0.8f
#define AGV_ARRIVAL_TOLERANCE_M     0.05f
#define AGV_STEERING_GAIN           1.0f

/* ---- EKF defaults ---- */
#define EKF_DT_S                    0.02f
#define EKF_INIT_P                  0.1f
#define EKF_Q_POS                   0.01f
#define EKF_Q_HEADING               0.004f
#define EKF_Q_VEL                   0.1f
#define EKF_Q_VEL_HEADING           0.05f

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
