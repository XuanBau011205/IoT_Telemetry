#include "sensor_logic.h"

#include <stdio.h>

uint8_t computeSmokeIndex(float mq2_median)
{
    if (mq2_median <= static_cast<float>(MQ2_BASELINE)) {
        return 0U;
    }

    if (mq2_median >= static_cast<float>(MQ2_DANGER_LEVEL)) {
        return 100U;
    }

    float normalized = (mq2_median - static_cast<float>(MQ2_BASELINE)) * 100.0f /
        static_cast<float>(MQ2_DANGER_LEVEL - MQ2_BASELINE);

    if (normalized <= 0.0f) {
        return 0U;
    }
    if (normalized >= 100.0f) {
        return 100U;
    }

    return static_cast<uint8_t>(normalized + 0.5f);
}

uint8_t computeSmokeSeverity(uint8_t smoke_index)
{
    if (smoke_index >= 75U) {
        return 3U;
    }
    if (smoke_index >= SMOKE_DETECT_INDEX) {
        return 2U;
    }
    if (smoke_index >= 25U) {
        return 1U;
    }
    return 0U;
}

bool computeSmokeDetected(uint8_t smoke_index)
{
    return smoke_index >= SMOKE_DETECT_INDEX;
}

uint8_t composeTelemetryFlags(bool temp_fired, bool hum_fired, bool smoke_detected)
{
    return (temp_fired ? FLAG_TEMP_FIRED : 0U) |
           (hum_fired ? FLAG_HUM_FIRED : 0U) |
           (smoke_detected ? FLAG_SMOKE_ALERT : 0U);
}

size_t formatTelemetryJsonDebug(char* out, size_t out_size, const SensorDebugSnapshot& snapshot)
{
    if (out == nullptr || out_size == 0U) {
        return 0U;
    }

    int written = snprintf(
        out,
        out_size,
        "{\"device_uptime_sec\":%lu,"
        "\"temperature_c\":%.2f,"
        "\"humidity_pct\":%.2f,"
        "\"node_id\":%u,"
        "\"flags\":%u,"
        "\"sequence\":%u,"
        "\"mq2_raw\":%u,"
        "\"mq2_median\":%.2f,"
        "\"smoke_index\":%u,"
        "\"smoke_detected\":%u,"
        "\"smoke_severity\":%u}",
        static_cast<unsigned long>(snapshot.device_uptime_sec),
        snapshot.temperature_c,
        snapshot.humidity_pct,
        static_cast<unsigned int>(snapshot.node_id),
        static_cast<unsigned int>(snapshot.flags),
        static_cast<unsigned int>(snapshot.sequence),
        static_cast<unsigned int>(snapshot.mq2_raw),
        snapshot.mq2_median,
        static_cast<unsigned int>(snapshot.smoke_index),
        static_cast<unsigned int>(snapshot.smoke_detected),
        static_cast<unsigned int>(snapshot.smoke_severity)
    );

    if (written < 0) {
        out[0] = '\0';
        return 0U;
    }

    if (static_cast<size_t>(written) >= out_size) {
        return out_size - 1U;
    }

    return static_cast<size_t>(written);
}
