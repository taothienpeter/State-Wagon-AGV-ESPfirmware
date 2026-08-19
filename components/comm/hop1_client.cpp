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

Hop1Client::Hop1Client(const char* vehicle_id, const char* host, uint16_t port, float max_speed_mps)
    : port_(port), max_speed_mps_(max_speed_mps), state_(Hop1State::DISCONNECTED),
      sock_(-1), send_mux_(nullptr)
{
    strncpy(vehicle_id_, vehicle_id, sizeof(vehicle_id_) - 1);
    vehicle_id_[sizeof(vehicle_id_) - 1] = 0;
    strncpy(host_, host, sizeof(host_) - 1);
    host_[sizeof(host_) - 1] = 0;
    send_mux_ = xSemaphoreCreateMutex();
}

Hop1Client::~Hop1Client() {
    if (send_mux_) {
        vSemaphoreDelete(send_mux_);
        send_mux_ = nullptr;
    }
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

    /* Send connection handshake with capabilities */
    char conn_payload[128];
    snprintf(conn_payload, sizeof(conn_payload),
             "{\"connectionState\":\"ONLINE\",\"capabilities\":{\"max_speed_mps\":%.2f}}",
             (double)max_speed_mps_);
    sendEnvelope(sock_, "connection", conn_payload);

    ESP_LOGI(TAG, "Connected to server %s:%u", host_, port_);
    return true;
}

void Hop1Client::handleIncomingLoop(int sock) {
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
        if (msg_len == 0 || msg_len > sizeof(recv_buf_) - 1) {
            ESP_LOGW(TAG, "Invalid message length: %lu", (unsigned long)msg_len);
            return;
        }

        /* Read JSON payload */
        r = recv(sock, (char*)recv_buf_, msg_len, MSG_WAITALL);
        if (r <= 0) {
            ESP_LOGW(TAG, "Connection lost (payload)");
            return;
        }
        recv_buf_[msg_len] = 0;

        /* Dispatch */
        incoming_doc_.clear();
        DeserializationError err = deserializeJson(incoming_doc_, (const char*)recv_buf_);
        if (err) {
            ESP_LOGW(TAG, "deserializeJson failed: %s", err.c_str());
            continue;
        }

        const char* type = incoming_doc_["type"];
        if (!type) continue;

        if (strcmp(type, "order") == 0 && order_cb_) {
            /* Extract orderId and waypoints from payload */
            JsonObject payload = incoming_doc_["payload"];
            const char* order_id = payload["orderId"] | "";
            /* Serialize waypoints array into scratch buffer (no malloc) */
            JsonArray wps = payload["waypoints"];
            if (wps.size() > 0) {
                size_t wps_len = measureJson(wps);
                if (wps_len < sizeof(wps_scratch_)) {
                    serializeJson(wps, wps_scratch_, sizeof(wps_scratch_));
                    order_cb_(order_id, wps_scratch_);
                } else {
                    ESP_LOGE(TAG, "Order waypoints JSON exceeds scratch buffer (%zu B)", wps_len);
                }
            } else {
                order_cb_(order_id, "[]");
            }
        } else if (strcmp(type, "instantAction") == 0 && action_cb_) {
            JsonObject payload = incoming_doc_["payload"];
            const char* action_type = payload["actionType"] | "";
            action_cb_(action_type);
        }
    }
}

void Hop1Client::sendEnvelope(int sock, const char* type, const char* payload) {
    if (sock < 0) return;

    if (send_mux_) xSemaphoreTake(send_mux_, portMAX_DELAY);

    static StaticJsonDocument<2048> doc;
    static StaticJsonDocument<1024> payload_doc;
    static char buf[2048];

    doc.clear();
    doc["type"] = type;
    doc["vehicleId"] = vehicle_id_;
    JsonObject payload_obj = doc.createNestedObject("payload");

    /* Parse the payload string into the envelope */
    payload_doc.clear();
    DeserializationError err = deserializeJson(payload_doc, payload);
    if (!err) {
        payload_obj.set(payload_doc.as<JsonObject>());
    }

    size_t written = serializeJson(doc, buf, sizeof(buf));
    if (written > 0 && written < sizeof(buf)) {
        sendFrame(sock, buf, written);
    }

    if (send_mux_) xSemaphoreGive(send_mux_);
}

void Hop1Client::sendFrame(int sock, const char* data, size_t len) {
    if (sock < 0 || !data || len == 0) return;

    uint8_t prefix[4];
    prefix[0] = (len >> 24) & 0xFF;
    prefix[1] = (len >> 16) & 0xFF;
    prefix[2] = (len >> 8) & 0xFF;
    prefix[3] = len & 0xFF;

    /* Send prefix */
    size_t total_sent = 0;
    while (total_sent < 4) {
        int ret = send(sock, (char*)prefix + total_sent, 4 - total_sent, 0);
        if (ret <= 0) return;
        total_sent += ret;
    }

    /* Send payload */
    total_sent = 0;
    while (total_sent < len) {
        int ret = send(sock, data + total_sent, len - total_sent, 0);
        if (ret <= 0) return;
        total_sent += ret;
    }
}

void Hop1Client::sendAck(const char* order_id, const char* status, const char* reason) {
    if (state_ != Hop1State::CONNECTED || sock_ < 0 || !order_id) return;

    StaticJsonDocument<256> ack_doc;
    ack_doc["orderId"] = order_id;
    ack_doc["status"] = status ? status : "ACCEPTED";
    if (reason && reason[0] != '\0') {
        ack_doc["reason"] = reason;
    }

    char payload[256];
    size_t len = serializeJson(ack_doc, payload, sizeof(payload));
    if (len > 0 && len < sizeof(payload)) {
        sendEnvelope(sock_, "ack", payload);
    }
}

void Hop1Client::publishState(const char* state_payload_json) {
    if (state_ != Hop1State::CONNECTED || sock_ < 0) return;
    sendEnvelope(sock_, "state", state_payload_json);
}

