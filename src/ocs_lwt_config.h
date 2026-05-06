#ifndef OCS_LWT_CONFIG_H
#define OCS_LWT_CONFIG_H

#include <stdint.h>

namespace OCS_LWT {

    // ─── DEVICE ────────────────────────────────────────────────
    static constexpr char CLIENT_ID[] = "ocs-node-01";

    // ─── TOPIC ─────────────────────────────────────────────────
    static constexpr char STATUS_TOPIC[] = "ocs/status";
    static constexpr char DATA_TOPIC[]   = "ocs/data";

    // ─── LWT PAYLOAD ───────────────────────────────────────────
    static constexpr char MSG_OFFLINE[] = "offline";
    static constexpr char MSG_ONLINE[]  = "online";

    // ─── MQTT CONFIG ───────────────────────────────────────────
    static constexpr uint8_t QOS    = 1;
    static constexpr bool    RETAIN = true;

}

#endif // OCS_LWT_CONFIG_H