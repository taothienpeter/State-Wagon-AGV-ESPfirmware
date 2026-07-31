#ifndef HOP1_CLIENT_H
#define HOP1_CLIENT_H

#include <stdint.h>
#include <functional>
#include <string>

#ifdef __cplusplus

/* Callback types */
typedef std::function<void(const char* order_id, const char* waypoints_json)> OrderCallback;
typedef std::function<void(const char* action_type)> ActionCallback;

enum class Hop1State {
    DISCONNECTED,
    CONNECTING,
    CONNECTED
};

class Hop1Client {
public:
    Hop1Client(const char* vehicle_id, const char* host, uint16_t port);

    /* Start the client task (pinned to core). Returns immediately. */
    void begin();

    /* Register callbacks for incoming messages */
    void onOrder(OrderCallback cb) { order_cb_ = cb; }
    void onInstantAction(ActionCallback cb) { action_cb_ = cb; }

    /* Publish state payload to server (wraps in envelope, thread-safe) */
    void publishState(const char* state_payload_json);

    /* Connection state accessors */
    Hop1State linkState() const { return state_; }
    bool isConnected() const { return state_ == Hop1State::CONNECTED; }

private:
    static void taskFunc(void* arg);
    void taskLoop();

    bool connectOnce();
    void handleIncomingLoop(int sock);
    void sendEnvelope(int sock, const char* type, const char* payload);
    void sendFrame(int sock, const char* data, size_t len);

    char vehicle_id_[32];
    char host_[64];
    uint16_t port_;
    Hop1State state_;
    int sock_;

    OrderCallback order_cb_;
    ActionCallback action_cb_;

    /* Stored JSON for publishing (locked by publish mutex) */
    char pending_json_[2048];
    bool has_pending_;
};

#endif /* __cplusplus */
#endif /* HOP1_CLIENT_H */
