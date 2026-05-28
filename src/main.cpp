// =============================================================================
// main.cpp — OCS Edge Node | ESP32 Industrial IoT Telemetry
// Architecture: Zero-heap, Non-blocking, Binary MQTT payload
// =============================================================================

#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

#include "OCS_RobustWindowFilter.h"
#include "ocs_firewall_v4.h"
#include "ocs_lwt_config.h"
#include "sensor_logic.h"

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
#define MQTT_BROKER "broker.hivemq.com"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
// #define ENABLE_MQTT_JSON_DEBUG 1
// =============================================================================
// TIMING CONSTANTS (milliseconds)
// =============================================================================
static constexpr uint32_t SENSOR_INTERVAL_MS   = 500U;
static constexpr uint32_t WIFI_RETRY_MS        = 5000U;
static constexpr uint32_t MQTT_RETRY_MS        = 3000U;
static constexpr uint32_t STATUS_INTERVAL_MS   = 60000U;  // Every 60 seconds
static constexpr uint32_t DHT_READ_INTERVAL_MS = 2000U;

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

// Robust window filters keep the existing median-window smoothing behavior.
static OCS_RobustWindowFilter s_temp_filter;
static OCS_RobustWindowFilter s_hum_filter;
static OCS_RobustWindowFilter s_mq2_filter;

// SoD Firewalls — one per channel
static ocs::SoDFirewall::Config s_temp_firewall_config = {
    .delta            = 0.25f,   // °C
    .heartbeat_sec    = 60U,     // force transmit every 60 s
    .min_interval_sec = 1U       // gate: no faster than 1 s
};
static ocs::SoDFirewall::State s_temp_firewall_state = { .version = 0 };

static ocs::SoDFirewall::Config s_hum_firewall_config = {
    .delta            = 0.5f,    // %RH
    .heartbeat_sec    = 60U,
    .min_interval_sec = 1U
};
static ocs::SoDFirewall::State s_hum_firewall_state = { .version = 0 };

// MQTT + WiFi
static WiFiClient   s_wifi_client;
static PubSubClient s_mqtt_client(s_wifi_client);

// Rolling frame counter
static uint16_t s_sequence = 0U;

// Non-blocking tick timestamps
static uint32_t s_last_sensor_ms  = 0U;
static uint32_t s_last_wifi_ms    = 0U;
static uint32_t s_last_mqtt_ms    = 0U;
static uint32_t s_last_status_ms  = 0U;
static uint32_t s_last_dht_ms     = 0U;
static uint32_t s_startup_ms      = 0U;

// Connection state tracking
static bool s_wifi_connected_prev = false;
static bool s_mqtt_connected_prev = false;

static_assert(MQ2_MEDIAN_WINDOW == WINDOW_SIZE,
    "MQ2_MEDIAN_WINDOW must match OCS_RobustWindowFilter WINDOW_SIZE.");

static uint16_t s_mq2_raw = 0U;
static float    s_mq2_median = 0.0f;
static uint8_t  s_smoke_index = 0U;
static bool     s_smoke_detected = false;
static uint8_t  s_smoke_severity = 0U;

// Binary telemetry is the production contract consumed by the backend.
// This JSON mirror is only for MQTT Explorer/manual inspection.
#if ENABLE_MQTT_JSON_DEBUG
static void publishTelemetryJsonDebug(const OcsPayload& payload)
{
    char json[320];
    SensorDebugSnapshot snapshot = {
        payload.timestamp_sec,
        payload.temperature,
        payload.humidity,
        payload.node_id,
        payload.flags,
        payload.sequence,
        s_mq2_raw,
        s_mq2_median,
        s_smoke_index,
        s_smoke_detected,
        s_smoke_severity
    };

    formatTelemetryJsonDebug(json, sizeof(json), snapshot);

    s_mqtt_client.publish(OCS_LWT::DEBUG_JSON_TOPIC, json);
}
#endif

// =============================================================================
// ANALOG SENSOR READ — replace with your actual sensor driver
// GPIO 34 = temperature analog, GPIO 35 = humidity analog (placeholder)
// =============================================================================
static constexpr uint8_t PIN_DHT22    = 4U;
static constexpr uint8_t PIN_MQ2_ADC  = 32U;

static DHT s_dht(PIN_DHT22, DHT22);

static bool  s_dht_has_valid_reading = false;
static float s_last_temperature_c = 25.0f;
static float s_last_humidity_pct = 50.0f;

static inline float read_temperature_raw()
{
    // Example: 12-bit ADC → 0–3.3 V → linear map to -40…85 °C
    return s_last_temperature_c;
}

static inline float read_humidity_raw()
{
    return s_last_humidity_pct;
}

static inline uint16_t read_mq2_raw()
{
    return static_cast<uint16_t>(analogRead(PIN_MQ2_ADC));
}

static bool read_dht22_safe(uint32_t now, float& temperature_c, float& humidity_pct)
{
    if (s_dht_has_valid_reading &&
        (uint32_t)(now - s_last_dht_ms) < DHT_READ_INTERVAL_MS) {
        temperature_c = s_last_temperature_c;
        humidity_pct = s_last_humidity_pct;
        return true;
    }

    s_last_dht_ms = now;
    float t = s_dht.readTemperature();
    float h = s_dht.readHumidity();

    if (!isnan(t) && !isnan(h)) {
        s_last_temperature_c = t;
        s_last_humidity_pct = h;
        s_dht_has_valid_reading = true;
        temperature_c = t;
        humidity_pct = h;
        return true;
    }

    temperature_c = s_last_temperature_c;
    humidity_pct = s_last_humidity_pct;
    return s_dht_has_valid_reading;
}

// =============================================================================
// DIAGNOSTIC LOGGING UTILITIES
// =============================================================================
static void log_mqtt_error(int8_t state)
{
    const char* err_msg = "Unknown";
    
    switch (state) {
        case -4: err_msg = "Connection timeout"; break;
        case -3: err_msg = "Connection lost"; break;
        case -2: err_msg = "Connection refused"; break;
        case -1: err_msg = "Disconnected"; break;
        case 0:  err_msg = "Connected"; break;
        case 1:  err_msg = "Bad protocol"; break;
        case 2:  err_msg = "Bad client ID"; break;
        case 3:  err_msg = "Server unavailable"; break;
        case 4:  err_msg = "Bad credentials"; break;
        case 5:  err_msg = "Not authorized"; break;
    }
    
    Serial.printf("[MQTT] State: %s (%d)\n", err_msg, state);
}

// =============================================================================
// WIFI — non-blocking reconnect with improved stability
// =============================================================================
static void wifi_task(uint32_t now)
{
    uint8_t wifi_status = WiFi.status();
    bool wifi_connected = (wifi_status == WL_CONNECTED);

    // Log state change
    if (wifi_connected != s_wifi_connected_prev) {
        s_wifi_connected_prev = wifi_connected;
        if (wifi_connected) {
            Serial.printf("[WiFi] ✓ Connected to '%s' (RSSI: %d dBm)\n", 
                WiFi.SSID().c_str(), WiFi.RSSI());
        } else {
            Serial.printf("[WiFi] ✗ Disconnected (status: %u)\n", wifi_status);
        }
    }

    if (wifi_connected) {
        return;
    }

    if ((uint32_t)(now - s_last_wifi_ms) < WIFI_RETRY_MS) {
        return;
    }

    s_last_wifi_ms = now;

    // Only disconnect if in unstable state (avoid radio thrashing)
    if (wifi_status != WL_IDLE_STATUS && wifi_status != WL_NO_SSID_AVAIL) {
        WiFi.disconnect(false);  // Don't power down radio; just disconnect
    }

    WiFi.mode(WIFI_STA);
    Serial.printf("[WiFi] Attempting connection to '%s'...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// =============================================================================
// MQTT — non-blocking reconnect with LWT and error reporting
// =============================================================================
static void mqtt_task(uint32_t now)
{
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    bool mqtt_connected = s_mqtt_client.connected();

    // Log state change
    if (mqtt_connected != s_mqtt_connected_prev) {
        s_mqtt_connected_prev = mqtt_connected;
        if (mqtt_connected) {
            Serial.println("[MQTT] ✓ Connected to broker");
        } else {
            Serial.printf("[MQTT] ✗ Disconnected\n");
            log_mqtt_error(s_mqtt_client.state());
        }
    }

    if (mqtt_connected) {
        s_mqtt_client.loop();
        return;
    }

    if ((uint32_t)(now - s_last_mqtt_ms) < MQTT_RETRY_MS) {
        return;
    }

    s_last_mqtt_ms = now;

    Serial.printf("[MQTT] Attempting connection to %s:%u...\n", 
        MQTT_BROKER, MQTT_PORT);

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
        Serial.println("[MQTT] Publishing online status...");
        
        // Announce online presence
        bool pub_ok = s_mqtt_client.publish(
            OCS_LWT::STATUS_TOPIC,
            reinterpret_cast<const uint8_t*>(OCS_LWT::MSG_ONLINE),
            sizeof(OCS_LWT::MSG_ONLINE) - 1U,  // strip null terminator
            OCS_LWT::RETAIN
        );
        
        if (!pub_ok) {
            Serial.println("[MQTT] ✗ Failed to publish online status");
        }
    } else {
        log_mqtt_error(s_mqtt_client.state());
    }
}

// =============================================================================
// STATUS & HEARTBEAT — periodic diagnostic publishing
// =============================================================================
static void status_task(uint32_t now)
{
    if ((uint32_t)(now - s_last_status_ms) < STATUS_INTERVAL_MS) {
        return;
    }

    s_last_status_ms = now;

    if (!s_mqtt_client.connected()) {
        return;
    }

    uint32_t uptime_sec = (now - s_startup_ms) / 1000U;
    int32_t rssi = WiFi.RSSI();

    // Publish uptime as diagnostic (topic: ocs/uptime)
    char uptime_str[32];
    snprintf(uptime_str, sizeof(uptime_str), "%u", uptime_sec);
    
    bool uptime_ok = s_mqtt_client.publish(
        OCS_LWT::UPTIME_TOPIC,
        reinterpret_cast<const uint8_t*>(uptime_str),
        strlen(uptime_str),
        false
    );

    // Publish heartbeat as counter
    char hb_str[16];
    snprintf(hb_str, sizeof(hb_str), "%u", s_sequence);
    
    bool hb_ok = s_mqtt_client.publish(
        OCS_LWT::HEARTBEAT_TOPIC,
        reinterpret_cast<const uint8_t*>(hb_str),
        strlen(hb_str),
        false
    );

    Serial.printf("[Status] Uptime: %u s | WiFi RSSI: %d dBm | Seq: %u | Topics: %s/%s\n",
        uptime_sec, rssi, s_sequence,
        uptime_ok ? "✓" : "✗",
        hb_ok ? "✓" : "✗");
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

    // 1. Read real sensors and filter
    float temperature_c;
    float humidity_pct;
    bool dht_ok = read_dht22_safe(now, temperature_c, humidity_pct);
    float temp_filtered = s_temp_filter.process(temperature_c);
    float hum_filtered  = s_hum_filter.process(humidity_pct);
    s_mq2_raw = read_mq2_raw();
    s_mq2_median = s_mq2_filter.process(static_cast<float>(s_mq2_raw));
    s_smoke_index = computeSmokeIndex(s_mq2_median);
    s_smoke_severity = computeSmokeSeverity(s_smoke_index);
    s_smoke_detected = computeSmokeDetected(s_smoke_index);

    if (!dht_ok) {
        Serial.println("[Sensor] DHT22 read failed; using last safe temperature/humidity values");
    }

    // 2. Firewall evaluation — seconds granularity for timestamps
    uint32_t now_sec = now / 1000U;

    bool temp_fired = ocs::SoDFirewall::should_transmit(
        temp_filtered, now_sec, s_temp_firewall_config, s_temp_firewall_state);

    bool hum_fired = ocs::SoDFirewall::should_transmit(
        hum_filtered, now_sec, s_hum_firewall_config, s_hum_firewall_state);

    if (!temp_fired && !hum_fired && !s_smoke_detected) {
        return;  // Nothing changed — suppress transmission
    }

    if (!s_mqtt_client.connected()) {
        Serial.printf("[Sensor] ⚠ MQTT not connected, frame dropped (seq=%u)\n", s_sequence);
        return;  // Drop frame; broker unreachable
    }

    // 3. Build packed 16-byte payload on the stack — zero heap
    OcsPayload payload;
    payload.timestamp_sec = now_sec;
    payload.temperature   = temp_filtered;
    payload.humidity      = hum_filtered;
    payload.node_id       = 0x01U;
    payload.flags         = composeTelemetryFlags(temp_fired, hum_fired, s_smoke_detected);
    payload.sequence      = s_sequence++;

    // 4. Transmit raw bytes — no JSON, no String, no heap
    // Binary telemetry is the production contract. Backend continues consuming it.
    bool pub_ok = s_mqtt_client.publish(
        OCS_LWT::DATA_TOPIC,
        reinterpret_cast<const uint8_t*>(&payload),
        sizeof(payload),
        OCS_LWT::QOS_DATA  // Use consistent QoS from config
    );

    // JSON debug is only a mirror for MQTT Explorer/manual inspection.
#if ENABLE_MQTT_JSON_DEBUG
    publishTelemetryJsonDebug(payload);
#endif

    if (!pub_ok) {
        Serial.printf("[Sensor] ✗ Publish FAILED (seq=%u, T=%.1f°C, H=%.1f%%)\n",
            payload.sequence - 1, temp_filtered, hum_filtered);
        s_sequence--;  // Rollback on failure
    } else {
        Serial.printf("[Sensor] ✓ Frame %u published (T=%.1f°C, H=%.1f%%, flags=0x%02x)\n",
            payload.sequence - 1, temp_filtered, hum_filtered, payload.flags);
    }
}

// =============================================================================
// ARDUINO ENTRY POINTS
// =============================================================================
#ifndef PIO_UNIT_TESTING
void setup()
{
    Serial.begin(115200);
    delay(500);  // Let serial stabilize

    Serial.println("\n========================================");
    Serial.println("OCS Edge Node — Telemetry System");
    Serial.printf("Build: %s %s\n", __DATE__, __TIME__);
    Serial.printf("Firmware: %s\n", OCS_LWT::FIRMWARE_VERSION);
    Serial.printf("Node ID: %s\n", OCS_LWT::CLIENT_ID);
    Serial.println("========================================\n");

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // 0–3.3 V range

    s_dht.begin();
    s_mqtt_client.setServer(MQTT_BROKER, MQTT_PORT);
#if ENABLE_MQTT_JSON_DEBUG
    s_mqtt_client.setBufferSize(384U); // Debug JSON mirror needs room for topic + JSON payload.
#else
    s_mqtt_client.setBufferSize(64U);  // header + 16-byte payload fits easily
#endif

    s_startup_ms = millis();

    Serial.println("[Setup] ✓ Initialization complete");
    Serial.printf("[Setup] WiFi SSID: %s\n", WIFI_SSID);
    Serial.printf("[Setup] MQTT Broker: %s:%u\n", MQTT_BROKER, MQTT_PORT);
    Serial.printf("[Setup] Topics: %s, %s, %s, %s\n",
        OCS_LWT::DATA_TOPIC,
        OCS_LWT::STATUS_TOPIC,
        OCS_LWT::UPTIME_TOPIC,
        OCS_LWT::HEARTBEAT_TOPIC);
}
#endif

#ifndef PIO_UNIT_TESTING
void loop()
{
    uint32_t now = millis();  // single read per loop — consistent timestamp

    wifi_task(now);
    mqtt_task(now);
    sensor_task(now);
    status_task(now);
}
#endif
