#ifndef OCS_LWT_CONFIG_H
#define OCS_LWT_CONFIG_H

#include <stdint.h>

namespace OCS_LWT {

    // ─── DEVICE ────────────────────────────────────────────────
    static constexpr char CLIENT_ID[] = "ocs-node-01";
    static constexpr char FIRMWARE_VERSION[] = "v1.0.0";

    // ─── TOPIC ─────────────────────────────────────────────────
    static constexpr char STATUS_TOPIC[] = "ocs/status";
    static constexpr char DATA_TOPIC[]   = "ocs/data";
    static constexpr char UPTIME_TOPIC[] = "ocs/uptime";
    static constexpr char HEARTBEAT_TOPIC[] = "ocs/heartbeat";

    // ─── LWT PAYLOAD ───────────────────────────────────────────
    static constexpr char MSG_OFFLINE[] = "offline";
    static constexpr char MSG_ONLINE[]  = "online";

    // ─── MQTT CONFIG ───────────────────────────────────────────
    static constexpr uint8_t QOS    = 1;
    static constexpr bool    RETAIN = true;
    static constexpr uint8_t QOS_DATA = QOS;  // Ensure data topic uses same QoS

}

#endif // OCS_LWT_CONFIG_H