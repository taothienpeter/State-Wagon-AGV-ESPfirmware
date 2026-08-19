#include "config.h"

#include "hop1_client.h"
#include "stm_uart_link.h"
#include "swerve_kinematics.h"
#include "trajectory_gen.h"
#include "bno055_driver.h"
#include "dwm1000_driver.h"
#include "es_ekf.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include <ArduinoJson.h>
#include <string.h>
#include <vector>

static const char* TAG = "AGV_MAIN";

/* =====================================================================
   Global shared state (protected by spinlock ekf_mux)
   ===================================================================== */
static StmUartLink    g_stm_link;
static Hop1Client*    g_hop1 = NULL;
static SwerveKinematics g_kinematics;
static TrajectoryGen  g_trajectory;
static ES_EKF         g_ekf;

/* EKF shared data protection */
static portMUX_TYPE   ekf_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile float g_pose_x = 0, g_pose_y = 0, g_pose_theta = 0;
static volatile float g_pose_vx = 0, g_pose_omega = 0;
static volatile float g_imu_heading = 0;
static volatile bool  g_imu_updated = false;
static volatile float g_uwb_x = 0, g_uwb_y = 0;
static volatile bool  g_uwb_updated = false;

/* Current order ID (set on order received, protected by g_order_mux) */
static char g_current_order_id[64] = "";
static portMUX_TYPE g_order_mux = portMUX_INITIALIZER_UNLOCKED;

/* Vehicle geometry for kinematics (no track_width in v2) */
static VehicleGeometry g_vehicle_geom = {
    .wheelbase_m = AGV_WHEELBASE_M,
    .wheel_radius_m = AGV_WHEEL_RADIUS_M,
    .max_velocity_mps = AGV_MAX_VELOCITY_MPS,
    .max_steering_angle_rad = AGV_MAX_STEERING_ANGLE_RAD,
    .max_steering_rate_radps = AGV_MAX_STEERING_RATE_RADPS,
};

/* =====================================================================
   Task trampolines (static functions → instance)
   ===================================================================== */

/* ---- Planner task (core 1, 50-80 Hz) ---- */
static void plannerTaskFunc(void* arg) {
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); /* 50 Hz */

    /* Wait for EKF to be initialized */
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        /* Compute actual dt from tick timing (see ARCHITECTUREv3 §7.5) */
        static TickType_t last_tick = 0;
        TickType_t now = xTaskGetTickCount();
        float dt_s = (last_tick == 0) ? 0.02f
                   : (now - last_tick) * portTICK_PERIOD_MS / 1000.0f;
        if (dt_s < 0.001f) dt_s = 0.02f;  /* clamp minimum */
        if (dt_s > 0.05f) dt_s = 0.05f;   /* clamp maximum (spurious wake) */
        last_tick = now;

        /* ---- Sensor fusion step ---- */

        /* 1. Read STM32 feedback for wheel odometry */
        motion_fb_t fb;
        float delta_x = 0, delta_y = 0, delta_theta = 0;
        bool has_fb = g_stm_link.getLatestFeedback(fb, 300);
        if (has_fb) {
            /* Bicycle-model odometry: front wheel steers, rear wheel fixed (0 rad).
               Produces correct angular velocity omega = v * sin(steer) / L. */
            float steer_angle = fb.steering_angle_actual_rad;
            float v_drive = fb.drive_velocity_actual_mps;
            float angles[2] = {steer_angle, 0.0f};
            float speeds[2] = {v_drive, v_drive};
            BodyVelocity odom = g_kinematics.fromWheelFeedback(angles, speeds);

            delta_x = odom.vx_mps * cosf(g_ekf.x_nom[2]) * dt_s;
            delta_y = odom.vx_mps * sinf(g_ekf.x_nom[2]) * dt_s;
            delta_theta = odom.omega_radps * dt_s;
        }

        /* 2. EKF predict (wheel odometry + dynamic dt_s) */
        g_ekf.predict(delta_x, delta_y, delta_theta, dt_s);

        /* 3. EKF update from IMU heading (Primary orientation source) */
        portENTER_CRITICAL(&ekf_mux);
        bool imu_updated = g_imu_updated;
        float imu_heading = g_imu_heading;
        if (imu_updated) g_imu_updated = false;
        portEXIT_CRITICAL(&ekf_mux);

        if (imu_updated) {
            g_ekf.updateIMU(imu_heading);
        }

#if AGV_ENABLE_UWB
        /* 4. EKF update from UWB (if enabled) */
        portENTER_CRITICAL(&ekf_mux);
        bool uwb_updated = g_uwb_updated;
        float uwb_x = g_uwb_x, uwb_y = g_uwb_y;
        if (uwb_updated) g_uwb_updated = false;
        portEXIT_CRITICAL(&ekf_mux);

        if (uwb_updated) {
            g_ekf.updateUWB(uwb_x, uwb_y, g_ekf.x_nom[2]);
        }
#endif

        /* 5. Publish EKF pose + velocity to shared globals */
        portENTER_CRITICAL(&ekf_mux);
        g_pose_x = g_ekf.x_nom[0];
        g_pose_y = g_ekf.x_nom[1];
        g_pose_theta = g_ekf.x_nom[2];
        g_pose_vx = g_ekf.x_nom[3];
        g_pose_omega = g_ekf.x_nom[5];
        portEXIT_CRITICAL(&ekf_mux);

        /* ---- Trajectory generation → BodyVelocity ---- */
        float pose[3] = {g_pose_x, g_pose_y, g_pose_theta};
        BodyVelocity body_v = g_trajectory.tick(pose, dt_s);

        /* ---- BodyVelocity → Swerve kinematics → wheel commands ---- */
        SwerveCommand wheels[2];
        bool has_cmd = g_kinematics.toSwerveCommand(body_v, dt_s, wheels);

        TrajStatus traj_status = g_trajectory.status();

        /* ---- Accumulate wheel turns for ODrive position control ---- */
        const float wheel_circumference = 2.0f * 3.14159265f * AGV_WHEEL_RADIUS_M;
        static float accumulated_turns = 0.0f;
        static bool position_synced = false;

        /* Sync accumulator with actual encoder position when starting or while idle */
        if (has_fb && (!position_synced || traj_status == TrajStatus::IDLE)) {
            accumulated_turns = fb.drive_pos_actual_turns;
            position_synced = true;
        }

        if (has_cmd && traj_status == TrajStatus::ACTIVE) {
            float wheel_rps = wheels[0].drive_velocity_mps / wheel_circumference;
            accumulated_turns += wheel_rps * dt_s;
        }

        /* ---- Send motion command to STM32 ---- */
        motion_cmd_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        if (traj_status == TrajStatus::ESTOP) {
            cmd.mode = MODE_SAFE_STOP;
        } else if (traj_status == TrajStatus::ACTIVE && has_cmd) {
            cmd.drive_pos_target_turns = accumulated_turns;
            cmd.steering_angle_rad = wheels[0].steering_angle_rad;
            cmd.steering_velocity_radps = wheels[0].steering_rate_radps;
            cmd.mode = MODE_POSITION;
            cmd.flags = 0x03; /* steering_enable | drive_enable */
        } else {
            cmd.mode = MODE_IDLE;
        }

        g_stm_link.sendMotionCmd(cmd);
    }
}

/* ---- IMU sensor task (core 0, 100 Hz) ---- */
static void imuTaskFunc(void* arg) {
    (void)arg;

    Bno055Driver imu(IMU_I2C_NUM, IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN,
                     IMU_I2C_CLOCK_HZ, IMU_I2C_ADDR);

    if (!imu.begin()) {
        ESP_LOGE(TAG, "BNO055 init failed — EKF uses odometry only");
    }

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / IMU_SAMPLE_RATE_HZ);

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        float heading_rad;
        if (imu.readHeadingRad(heading_rad)) {
            portENTER_CRITICAL(&ekf_mux);
            g_imu_heading = heading_rad;
            g_imu_updated = true;
            portEXIT_CRITICAL(&ekf_mux);
        }
    }
}

#if AGV_ENABLE_UWB
/* ---- UWB ranging task (core 0, 10 Hz) ---- */
static void uwbTaskFunc(void* arg) {
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(200));

    Dwm1000Driver uwb(UWB_SPI_HOST, UWB_SPI_CS, UWB_SPI_IRQ, -1,
                      UWB_SPI_MOSI, UWB_SPI_MISO, UWB_SPI_CLK);
    bool uwb_available = uwb.begin();
    if (!uwb_available) {
        ESP_LOGW(TAG, "DWM1000 init skipped/failed — EKF uses odometry + IMU only");
    }

    const float anchors[4][3] = {
        {0.0f, 0.0f, 0.0f},
        {10.0f, 0.0f, 0.0f},
        {0.0f, 10.0f, 0.0f},
        {10.0f, 10.0f, 0.0f},
    };
    uwb.setAnchors(anchors, 4);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / UWB_RATE_HZ);

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        if (uwb_available) {
            float ranges[MAX_ANCHORS];
            int count = 0;
            if (uwb.doRanging(ranges, count) && count >= 3) {
                float x, y;
                if (uwb.trilaterate(ranges, count, x, y)) {
                    portENTER_CRITICAL(&ekf_mux);
                    g_uwb_x = x;
                    g_uwb_y = y;
                    g_uwb_updated = true;
                    portEXIT_CRITICAL(&ekf_mux);
                }
            }
        }
    }
}
#endif

/* ---- State publish task (core 0, 8 Hz) ---- */
static void statePublishTaskFunc(void* arg) {
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / AGV_STATE_PUBLISH_HZ);

    static StaticJsonDocument<1536> doc;
    static char json_buf[1536];

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        if (!g_hop1 || !g_hop1->isConnected()) continue;

        /* Read shared EKF pose */
        portENTER_CRITICAL(&ekf_mux);
        float px = g_pose_x, py = g_pose_y, pt = g_pose_theta;
        float vx = g_pose_vx, omega = g_pose_omega;
        float imu_h = g_imu_heading;
#if AGV_ENABLE_UWB
        float uwb_x = g_uwb_x, uwb_y = g_uwb_y;
#endif
        portEXIT_CRITICAL(&ekf_mux);

        /* Read STM32 feedback for drive controller status */
        motion_fb_t fb;
        bool fb_valid = g_stm_link.getLatestFeedback(fb, 500);
        bool link_ok = g_stm_link.linkOk();

        /* Read current orderId safely */
        char current_order_id[64];
        portENTER_CRITICAL(&g_order_mux);
        strncpy(current_order_id, g_current_order_id, sizeof(current_order_id) - 1);
        current_order_id[sizeof(current_order_id) - 1] = 0;
        portEXIT_CRITICAL(&g_order_mux);

        /* Determine operating mode and safety state */
        TrajStatus ts = g_trajectory.status();
        const char* operating_mode = (ts == TrajStatus::ESTOP) ? "MANUAL" : "AUTOMATIC";

        const char* safety_state_str = "NORMAL";
        if (ts == TrajStatus::ESTOP) {
            safety_state_str = "SAFE_STOP";
        } else {
            uint8_t safety_state = fb_valid ? fb.safety_state : 2; /* SAFE_STOP if no FB */
            safety_state_str = (safety_state == 0) ? "NORMAL" :
                               (safety_state == 1) ? "WARN" :
                               (safety_state == 2) ? "SAFE_STOP" : "FAULT_LATCHED";
        }

        /* Build state JSON payload (zero-malloc) */
        doc.clear();

        doc["orderId"] = (current_order_id[0] != '\0') ? current_order_id : nullptr;
        doc["orderState"] = g_trajectory.orderStateString();
        doc["operatingMode"] = operating_mode;
        doc["safetyState"] = safety_state_str;

        JsonObject pos = doc.createNestedObject("position");
        pos["x"] = px;
        pos["y"] = py;
        pos["theta"] = pt;

        JsonObject vel = doc.createNestedObject("velocity");
        vel["vx"] = vx;
        vel["vy"] = 0.0f;
        vel["omega"] = omega;

        JsonObject bat = doc.createNestedObject("battery");
        bat["percentage"] = 85.0f; /* placeholder — implement ADC reading */
        bat["voltage"] = fb_valid ? fb.odrive_vbus_v : 0.0f;
        bat["charging"] = false;

        JsonObject imu_obj = doc.createNestedObject("imu");
        JsonArray accel = imu_obj.createNestedArray("accel");
        accel.add(0.0f);
        accel.add(0.0f);
        accel.add(9.8f);
        JsonArray gyro = imu_obj.createNestedArray("gyro");
        gyro.add(0.0f);
        gyro.add(0.0f);
        gyro.add(0.0f);
        imu_obj["heading_rad"] = imu_h;
        imu_obj["calibration_status"] = 3; /* placeholder — read full cal status */

        JsonObject uwb_obj = doc.createNestedObject("uwb");
#if AGV_ENABLE_UWB
        uwb_obj["anchors"] = UWB_ANCHOR_COUNT;
        uwb_obj["quality"] = 0.0f;
        uwb_obj["x"] = uwb_x;
        uwb_obj["y"] = uwb_y;
#else
        uwb_obj["anchors"] = 0;
        uwb_obj["quality"] = 0.0f;
        uwb_obj["x"] = px;
        uwb_obj["y"] = py;
#endif

        JsonArray controllers = doc.createNestedArray("driveControllers");
        JsonObject ctrl = controllers.createNestedObject();
        ctrl["id"] = "stm32-main";
        ctrl["linkOk"] = link_ok;
        ctrl["faultCode"] = fb_valid ? fb.fault_code : 0;
        ctrl["odriveVbus"] = fb_valid ? fb.odrive_vbus_v : 0.0f;
        ctrl["odriveErrors"] = fb_valid ? fb.odrive_error_flags : 0;
        ctrl["stepperHomed"] = fb_valid ? fb.stepper_homed : 0;

        doc.createNestedArray("errors");

        size_t len = serializeJson(doc, json_buf, sizeof(json_buf));
        if (len > 0 && len < sizeof(json_buf)) {
            g_hop1->publishState(json_buf);
        }
    }
}

/* ---- Heartbeat task (core 0, 1 Hz) ---- */
static void heartbeatTaskFunc(void* arg) {
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(AGV_HEARTBEAT_PERIOD_MS);

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        if (g_hop1 && g_hop1->isConnected()) {
            ESP_LOGD(TAG, "Heartbeat — EKF: (%.2f, %.2f, %.2f)",
                     (double)g_pose_x, (double)g_pose_y, (double)g_pose_theta);
        }
    }
}

/* =====================================================================
   Callbacks
   ===================================================================== */
static void onOrderReceived(const char* order_id, const char* waypoints_json) {
    ESP_LOGI(TAG, "Order received: %s", order_id ? order_id : "(null)");

    static StaticJsonDocument<16384> order_doc;
    order_doc.clear();
    DeserializationError err = deserializeJson(order_doc, waypoints_json);
    if (err) {
        ESP_LOGE(TAG, "Failed to parse waypoints: %s", err.c_str());
        if (g_hop1) {
            g_hop1->sendAck(order_id, "REJECTED", "JSON parse error");
        }
        return;
    }

    JsonArray arr = order_doc.as<JsonArray>();
    if (arr.isNull() || arr.size() == 0) {
        ESP_LOGW(TAG, "Empty waypoint list — rejecting order");
        if (g_hop1) {
            g_hop1->sendAck(order_id, "REJECTED", "Empty waypoint list");
        }
        return;
    }

    std::vector<Waypoint> wps;
    wps.reserve(arr.size());
    for (JsonObject wp : arr) {
        Waypoint w;
        w.x = wp["x"] | 0.0f;
        w.y = wp["y"] | 0.0f;
        w.max_speed_mps = wp["max_speed_mps"] | 1.0f;
        w.tolerance_m = wp["tolerance_m"] | 0.05f;
        wps.push_back(w);
    }

    /* Store current order ID */
    portENTER_CRITICAL(&g_order_mux);
    strncpy(g_current_order_id, order_id, sizeof(g_current_order_id) - 1);
    g_current_order_id[sizeof(g_current_order_id) - 1] = 0;
    portEXIT_CRITICAL(&g_order_mux);

    g_trajectory.loadWaypoints(wps);

    /* Send ACCEPTED ack */
    if (g_hop1) {
        g_hop1->sendAck(order_id, "ACCEPTED");
    }
}

static void onInstantAction(const char* action_type) {
    ESP_LOGI(TAG, "Instant action: %s", action_type);

    if (strcmp(action_type, "emergencyStop") == 0) {
        g_trajectory.emergencyStop();
        /* Set STM32 to safe stop directly */
        motion_cmd_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.mode = MODE_SAFE_STOP;
        g_stm_link.sendMotionCmd(cmd);
    } else if (strcmp(action_type, "startPause") == 0) {
        g_trajectory.pause();
    } else if (strcmp(action_type, "stopPause") == 0) {
        g_trajectory.resume();
    } else if (strcmp(action_type, "clearFault") == 0) {
        g_trajectory.clearEstop();
    }
}

/* =====================================================================
   WiFi helpers
   ===================================================================== */
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strncpy((char*)wifi_config.sta.ssid, AGV_DEFAULT_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, AGV_DEFAULT_WIFI_PASS, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi: %s", AGV_DEFAULT_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(AGV_WIFI_CONNECT_TIMEOUT_MS));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected");
    } else {
        ESP_LOGW(TAG, "WiFi timeout — continuing without network");
    }
}

/* =====================================================================
   app_main — Entry Point
   ===================================================================== */
extern "C" void app_main(void) {
    /* ---- Init NVS ---- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=== AGV Swerve Drive v2 ===");

    /* Compile-time protocol struct size verification */
    static_assert(sizeof(motion_cmd_t) == 20, "motion_cmd_t must be 20 bytes");
    static_assert(sizeof(motion_fb_t) == 29, "motion_fb_t must be 29 bytes");

    /* ---- Init WiFi ---- */
    wifi_init_sta();

    /* ---- Init STM32 UART link ---- */
    g_stm_link.begin(STM_UART_NUM, STM_TX_PIN, STM_RX_PIN,
                     STM_UART_BAUD, STM_CMD_RATE_HZ);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* ---- Send STM32 config once at boot ---- */
    config_set_t cfg;
    cfg.drive_wheel_radius_m = AGV_WHEEL_RADIUS_M;
    cfg.wheelbase_m = AGV_WHEELBASE_M;
    cfg.max_drive_velocity_mps = AGV_MAX_VELOCITY_MPS;
    cfg.max_steering_angle_rad = AGV_MAX_STEERING_ANGLE_RAD;
    g_stm_link.sendConfig(cfg);

    /* ---- Init EKF ---- */
    g_ekf.init(EKF_DT_S, 0.0f, 0.0f, 0.0f);

    /* ---- Init kinematics ---- */
    g_kinematics = SwerveKinematics(g_vehicle_geom);

    /* ---- Init Hop1 client ---- */
    g_hop1 = new Hop1Client(AGV_DEFAULT_VEHICLE_ID, AGV_DEFAULT_SERVER_HOST, AGV_DEFAULT_SERVER_PORT, AGV_MAX_VELOCITY_MPS);
    g_hop1->onOrder(onOrderReceived);
    g_hop1->onInstantAction(onInstantAction);
    g_hop1->begin();

    /* ---- Create FreeRTOS tasks ---- */

    /* Planner: core 1, priority 10, 50 Hz */
    xTaskCreatePinnedToCore(&plannerTaskFunc, "planner", 6144, NULL, 10, NULL, 1);

    /* IMU sensor: core 0, priority 8, 100 Hz */
    xTaskCreatePinnedToCore(&imuTaskFunc, "imu_sensor", 4096, NULL, 8, NULL, 0);

#if AGV_ENABLE_UWB
    /* UWB ranging: core 0, priority 5, 10 Hz */
    xTaskCreatePinnedToCore(&uwbTaskFunc, "uwb_ranging", 4096, NULL, 5, NULL, 0);
#else
    ESP_LOGI(TAG, "UWB disabled — Localization running in IMU Yaw + Wheel Odometry mode");
#endif

    /* State publish: core 0, priority 5, 8 Hz */
    xTaskCreatePinnedToCore(&statePublishTaskFunc, "state_pub", 6144, NULL, 5, NULL, 0);

    /* Heartbeat: core 0, priority 3, 1 Hz */
    xTaskCreatePinnedToCore(&heartbeatTaskFunc, "heartbeat", 2048, NULL, 3, NULL, 0);

    ESP_LOGI(TAG, "All tasks created. System running.");
}
