// =============================================================================
// main.cpp — OCS Edge Node | ESP32 Industrial IoT Telemetry
// Architecture: Zero-heap, Non-blocking, Binary MQTT payload
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "ocs_median.h"
#include "ocs_firewall_v4.h"
#include "ocs_lwt_config.h"

// =============================================================================
// NETWORK CREDENTIALS — override via build flags in platformio.ini
// =============================================================================
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_SSID"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_PASSWORD"
#endif

#ifndef MQTT_BROKER
#define MQTT_BROKER "192.168.1.100"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

// =============================================================================
// TIMING CONSTANTS (milliseconds)
// =============================================================================
static constexpr uint32_t SENSOR_INTERVAL_MS   = 500U;
static constexpr uint32_t WIFI_RETRY_MS        = 5000U;
static constexpr uint32_t MQTT_RETRY_MS        = 3000U;

// =============================================================================
// BINARY PAYLOAD — exactly 16 bytes, packed, zero padding
// Layout:
//   [0..3]  uint32_t  timestamp_sec   — Unix epoch (seconds)
//   [4..7]  float     temperature     — °C, IEEE-754
//   [8..11] float     humidity        — %, IEEE-754
//   [12]    uint8_t   node_id         — device identifier
//   [13]    uint8_t   flags           — bit0=temp_fired, bit1=hum_fired
//   [14..15] uint16_t sequence        — rolling frame counter
// =============================================================================
struct __attribute__((packed)) OcsPayload {
    uint32_t timestamp_sec;
    float    temperature;
    float    humidity;
    uint8_t  node_id;
    uint8_t  flags;
    uint16_t sequence;
};

static_assert(sizeof(OcsPayload) == 16U,
    "OcsPayload MUST be exactly 16 bytes — check struct members and packing.");

// =============================================================================
// STATIC INSTANCES — zero heap, all state lives in .bss / .data
// =============================================================================

// Median filters
static OcsMedianFilter s_temp_filter;
static OcsMedianFilter s_hum_filter;

// SoD Firewalls — one per channel
static ocs::SoDFirewall::Config s_temp_fw_cfg = {
    .delta            = 0.25f,   // °C
    .heartbeat_sec    = 60U,     // force transmit every 60 s
    .min_interval_sec = 1U       // gate: no faster than 1 s
};
static ocs::SoDFirewall::State s_temp_fw_state = { .version = 0 };

static ocs::SoDFirewall::Config s_hum_fw_cfg = {
    .delta            = 0.5f,    // %RH
    .heartbeat_sec    = 60U,
    .min_interval_sec = 1U
};
static ocs::SoDFirewall::State s_hum_fw_state = { .version = 0 };

// MQTT + WiFi
static WiFiClient   s_wifi_client;
static PubSubClient s_mqtt_client(s_wifi_client);

// Rolling frame counter
static uint16_t s_sequence = 0U;

// Non-blocking tick timestamps
static uint32_t s_last_sensor_ms  = 0U;
static uint32_t s_last_wifi_ms    = 0U;
static uint32_t s_last_mqtt_ms    = 0U;

// =============================================================================
// ANALOG SENSOR READ — replace with your actual sensor driver
// GPIO 34 = temperature analog, GPIO 35 = humidity analog (placeholder)
// =============================================================================
static constexpr uint8_t PIN_TEMP_ADC = 34U;
static constexpr uint8_t PIN_HUM_ADC  = 35U;

static inline float read_temperature_raw()
{
    // Example: 12-bit ADC → 0–3.3 V → linear map to -40…85 °C
    uint16_t raw = static_cast<uint16_t>(analogRead(PIN_TEMP_ADC));
    return -40.0f + (static_cast<float>(raw) / 4095.0f) * 125.0f;
}

static inline float read_humidity_raw()
{
    uint16_t raw = static_cast<uint16_t>(analogRead(PIN_HUM_ADC));
    return (static_cast<float>(raw) / 4095.0f) * 100.0f;
}

// =============================================================================
// WIFI — non-blocking reconnect
// =============================================================================
static void wifi_task(uint32_t now)
{
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }

    if ((uint32_t)(now - s_last_wifi_ms) < WIFI_RETRY_MS) {
        return;
    }

    s_last_wifi_ms = now;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    // Connection is polled next iteration — no blocking waitForConnectResult()
}

// =============================================================================
// MQTT — non-blocking reconnect with LWT
// =============================================================================
static void mqtt_task(uint32_t now)
{
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    if (s_mqtt_client.connected()) {
        s_mqtt_client.loop();
        return;
    }

    if ((uint32_t)(now - s_last_mqtt_ms) < MQTT_RETRY_MS) {
        return;
    }

    s_last_mqtt_ms = now;

    // Establish connection with LWT — constants from ocs_lwt_config.h
    bool connected = s_mqtt_client.connect(
        OCS_LWT::CLIENT_ID,             // clientId
        nullptr,                        // username
        nullptr,                        // password
        OCS_LWT::STATUS_TOPIC,          // willTopic
        OCS_LWT::QOS,                   // willQos
        OCS_LWT::RETAIN,                // willRetain
        OCS_LWT::MSG_OFFLINE            // willMessage
    );

    if (connected) {
        // Announce online presence
        s_mqtt_client.publish(
            OCS_LWT::STATUS_TOPIC,
            reinterpret_cast<const uint8_t*>(OCS_LWT::MSG_ONLINE),
            sizeof(OCS_LWT::MSG_ONLINE) - 1U,  // strip null terminator
            OCS_LWT::RETAIN
        );
    }
}

// =============================================================================
// SENSOR + FIREWALL + PUBLISH — non-blocking, called on interval
// =============================================================================
static void sensor_task(uint32_t now)
{
    if ((uint32_t)(now - s_last_sensor_ms) < SENSOR_INTERVAL_MS) {
        return;
    }

    s_last_sensor_ms = now;

    // 1. Read raw ADC and filter
    float temp_filtered = s_temp_filter.process(read_temperature_raw());
    float hum_filtered  = s_hum_filter.process(read_humidity_raw());

    // 2. Firewall evaluation — seconds granularity for timestamps
    uint32_t now_sec = now / 1000U;

    bool temp_fired = ocs::SoDFirewall::should_transmit(
        temp_filtered, now_sec, s_temp_fw_cfg, s_temp_fw_state);

    bool hum_fired = ocs::SoDFirewall::should_transmit(
        hum_filtered, now_sec, s_hum_fw_cfg, s_hum_fw_state);

    if (!temp_fired && !hum_fired) {
        return;  // Nothing changed — suppress transmission
    }

    if (!s_mqtt_client.connected()) {
        return;  // Drop frame; broker unreachable
    }

    // 3. Build packed 16-byte payload on the stack — zero heap
    OcsPayload payload;
    payload.timestamp_sec = now_sec;
    payload.temperature   = temp_filtered;
    payload.humidity      = hum_filtered;
    payload.node_id       = 0x01U;
    payload.flags         = (static_cast<uint8_t>(temp_fired) << 0U) |
                            (static_cast<uint8_t>(hum_fired)  << 1U);
    payload.sequence      = s_sequence++;

    // 4. Transmit raw bytes — no JSON, no String, no heap
    s_mqtt_client.publish(
        OCS_LWT::DATA_TOPIC,
        reinterpret_cast<const uint8_t*>(&payload),
        sizeof(payload),
        false   // retain = false for live telemetry frames
    );
}

// =============================================================================
// ARDUINO ENTRY POINTS
// =============================================================================
void setup()
{
    Serial.begin(115200);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // 0–3.3 V range

    s_mqtt_client.setServer(MQTT_BROKER, MQTT_PORT);
    s_mqtt_client.setBufferSize(64U);  // header + 16-byte payload fits easily

    // Firewall states self-initialise on first call via version sentinel check
    // (OCS_FIREWALL_STATE_VERSION). No explicit init needed here.
}

void loop()
{
    uint32_t now = millis();  // single read per loop — consistent timestamp

    wifi_task(now);
    mqtt_task(now);
    sensor_task(now);
}