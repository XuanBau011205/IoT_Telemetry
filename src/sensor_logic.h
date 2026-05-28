#ifndef SENSOR_LOGIC_H
#define SENSOR_LOGIC_H

#include <stddef.h>
#include <stdint.h>

static constexpr uint8_t FLAG_TEMP_FIRED  = (1U << 0U);
static constexpr uint8_t FLAG_HUM_FIRED   = (1U << 1U);
static constexpr uint8_t FLAG_SMOKE_ALERT = (1U << 2U);

static constexpr uint8_t  MQ2_MEDIAN_WINDOW  = 5U;
static constexpr uint16_t MQ2_BASELINE       = 800U;
static constexpr uint16_t MQ2_DANGER_LEVEL   = 2500U;
static constexpr uint8_t  SMOKE_DETECT_INDEX = 50U;

struct SensorDebugSnapshot {
    uint32_t device_uptime_sec;
    float temperature_c;
    float humidity_pct;
    uint8_t node_id;
    uint8_t flags;
    uint16_t sequence;
    uint16_t mq2_raw;
    float mq2_median;
    uint8_t smoke_index;
    bool smoke_detected;
    uint8_t smoke_severity;
};

uint8_t computeSmokeIndex(float mq2_median);
uint8_t computeSmokeSeverity(uint8_t smoke_index);
bool computeSmokeDetected(uint8_t smoke_index);
uint8_t composeTelemetryFlags(bool temp_fired, bool hum_fired, bool smoke_detected);
size_t formatTelemetryJsonDebug(char* out, size_t out_size, const SensorDebugSnapshot& snapshot);

#endif // SENSOR_LOGIC_H
