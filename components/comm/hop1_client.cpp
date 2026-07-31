#include "hop1_client.h"
#include <ArduinoJson.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>
#include <cstdio>

static const char* TAG = "Hop1Client";

Hop1Client::Hop1Client(const char* vehicle_id, const char* host, uint16_t port)
    : port_(port), state_(Hop1State::DISCONNECTED), sock_(-1), has_pending_(false)
{
    strncpy(vehicle_id_, vehicle_id, sizeof(vehicle_id_) - 1);
    vehicle_id_[sizeof(vehicle_id_) - 1] = 0;
    strncpy(host_, host, sizeof(host_) - 1);
    host_[sizeof(host_) - 1] = 0;
}

void Hop1Client::begin() {
    xTaskCreatePinnedToCore(&taskFunc, "hop1_task", 8192, this, 6, NULL, 0);
}

void Hop1Client::taskFunc(void* arg) {
    Hop1Client* self = static_cast<Hop1Client*>(arg);
    self->taskLoop();
}

void Hop1Client::taskLoop() {
    int backoff_ms = 500;

    while (1) {
        switch (state_) {
        case Hop1State::DISCONNECTED:
            state_ = Hop1State::CONNECTING;
            backoff_ms = 500;
            break;

        case Hop1State::CONNECTING:
            if (connectOnce()) {
                state_ = Hop1State::CONNECTED;
                backoff_ms = 500;
                handleIncomingLoop(sock_);
                /* Disconnected: close and retry */
                if (sock_ >= 0) {
                    close(sock_);
                    sock_ = -1;
                }
                state_ = Hop1State::DISCONNECTED;
            } else {
                vTaskDelay(pdMS_TO_TICKS(backoff_ms));
                backoff_ms *= 2;
                if (backoff_ms > 15000) backoff_ms = 15000;
            }
            break;

        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
}

bool Hop1Client::connectOnce() {
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%u", port_);
    int err = getaddrinfo(host_, port_str, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGE(TAG, "DNS lookup failed for %s", host_);
        return false;
    }

    int s = socket(res->ai_family, res->ai_socktype, 0);
    if (s < 0) {
        freeaddrinfo(res);
        return false;
    }

    struct timeval timeout = {5, 0};
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (connect(s, res->ai_addr, res->ai_addrlen) < 0) {
        close(s);
        freeaddrinfo(res);
        return false;
    }
    freeaddrinfo(res);
    sock_ = s;

    /* Send connection handshake */
    sendEnvelope(sock_, "connection", "{\"connectionState\":\"ONLINE\"}");

    ESP_LOGI(TAG, "Connected to server %s:%u", host_, port_);
    return true;
}

void Hop1Client::handleIncomingLoop(int sock) {
    uint8_t buf[4096];

    while (1) {
        /* Read 4-byte length prefix */
        uint8_t len_buf[4];
        int r = recv(sock, (char*)len_buf, 4, MSG_WAITALL);
        if (r <= 0) {
            ESP_LOGW(TAG, "Connection lost (length prefix)");
            return;
        }
        uint32_t msg_len = (uint32_t)len_buf[0] << 24
                         | (uint32_t)len_buf[1] << 16
                         | (uint32_t)len_buf[2] << 8
                         | (uint32_t)len_buf[3];
        if (msg_len == 0 || msg_len > sizeof(buf) - 1) {
            ESP_LOGW(TAG, "Invalid message length: %lu", (unsigned long)msg_len);
            return;
        }

        /* Read JSON payload */
        r = recv(sock, (char*)buf, msg_len, MSG_WAITALL);
        if (r <= 0) {
            ESP_LOGW(TAG, "Connection lost (payload)");
            return;
        }
        buf[msg_len] = 0;

        /* Dispatch */
        DynamicJsonDocument doc(4096);
        DeserializationError err = deserializeJson(doc, (const char*)buf);
        if (err) continue;

        const char* type = doc["type"];
        if (!type) continue;

        if (strcmp(type, "order") == 0 && order_cb_) {
            /* Extract orderId and waypoints from payload */
            JsonObject payload = doc["payload"];
            const char* order_id = payload["orderId"] | "";
            /* Serialize waypoints array back to JSON string for the callback */
            JsonArray wps = payload["waypoints"];
            if (wps.size() > 0) {
                size_t wps_len = measureJson(wps) + 1;
                char* wps_json = (char*)malloc(wps_len);
                if (wps_json) {
                    serializeJson(wps, wps_json, wps_len);
                    order_cb_(order_id, wps_json);
                    free(wps_json);
                }
            } else {
                order_cb_(order_id, "[]");
            }
        } else if (strcmp(type, "instantAction") == 0 && action_cb_) {
            JsonObject payload = doc["payload"];
            const char* action_type = payload["actionType"] | "";
            action_cb_(action_type);
        }
    }
}

void Hop1Client::sendEnvelope(int sock, const char* type, const char* payload) {
    DynamicJsonDocument doc(2048);
    doc["type"] = type;
    doc["vehicleId"] = vehicle_id_;
    JsonObject payload_obj = doc.createNestedObject("payload");

    /* Parse the payload string into the envelope */
    DynamicJsonDocument payload_doc(1024);
    DeserializationError err = deserializeJson(payload_doc, payload);
    if (!err) {
        payload_obj.set(payload_doc.as<JsonObject>());
    }

    size_t len = measureJson(doc) + 1;
    char* buf = (char*)malloc(len);
    if (buf) {
        serializeJson(doc, buf, len);
        sendFrame(sock, buf, strlen(buf));
        free(buf);
    }
}

void Hop1Client::sendFrame(int sock, const char* data, size_t len) {
    uint8_t prefix[4];
    prefix[0] = (len >> 24) & 0xFF;
    prefix[1] = (len >> 16) & 0xFF;
    prefix[2] = (len >> 8) & 0xFF;
    prefix[3] = len & 0xFF;

    send(sock, (char*)prefix, 4, 0);
    send(sock, data, len, 0);
}

void Hop1Client::publishState(const char* state_payload_json) {
    if (state_ != Hop1State::CONNECTED || sock_ < 0) return;
    sendEnvelope(sock_, "state", state_payload_json);
}
