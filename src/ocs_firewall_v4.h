#ifndef OCS_TRANSMISSION_FIREWALL_V4_H
#define OCS_TRANSMISSION_FIREWALL_V4_H

#include <cstdint>
#include <cmath>

namespace ocs {

// CONFIG
#ifndef OCS_FIREWALL_STATE_VERSION
#define OCS_FIREWALL_STATE_VERSION 0xA5A5A5A5
#endif

#define OCS_TIME_DIFF_32(now, last) ((uint32_t)((now) - (last)))

// BASE (CRTP)
template<typename Derived>
class FirewallBase {
public:
    using T = float;

    // SỬA TẠI ĐÂY: Thêm template cho hàm để trì hoãn việc kiểm tra kiểu (Deferred Instantiation)
    template <typename CfgType, typename StateType>
    __attribute__((always_inline)) // Xóa chữ 'hot' đi nếu trình biên dịch ESP32 cảnh báo
    static bool should_transmit(
        T value,
        uint32_t now,
        const CfgType& cfg,
        StateType& state)
    {
        if (__builtin_expect(state.version != OCS_FIREWALL_STATE_VERSION, 0)) {
            Derived::reset_state(state);
        }

        if (__builtin_expect(!__builtin_isfinite(value), 0)) {
            return false;
        }

        if (state.initialized && cfg.min_interval_sec > 0) {
            // Ép kiểu now về uint32_t để đảm bảo macro hoạt động đúng
            if (OCS_TIME_DIFF_32(now, state.last_sent_timestamp) < cfg.min_interval_sec) {
                return false;
            }
        }

        return Derived::impl(value, now, cfg, state);
    }
};

// UTIL
static inline float ocs_absf(float x) {
    return __builtin_fabsf(x);
}

// ============================================================================
// SoD FIREWALL (cơ bản)
// ============================================================================
struct SoDFirewall : public FirewallBase<SoDFirewall> {
public:
    struct Config {
        float    delta;
        uint32_t heartbeat_sec;
        uint32_t min_interval_sec;
    };

    struct State {
        uint32_t version;
        float    last_value;
        uint32_t last_sent_timestamp;
        bool     initialized;
    };

    static void reset_state(State& s) {
        s.version = OCS_FIREWALL_STATE_VERSION;
        s.initialized = false;
    }

    static bool impl(float v, uint32_t t, const Config& cfg, State& s)
    {
        if (!s.initialized) {
            s.last_value = v;
            s.last_sent_timestamp = t;
            s.initialized = true;
            return true;
        }

        uint32_t dt = OCS_TIME_DIFF_32(t, s.last_sent_timestamp);

        if (cfg.heartbeat_sec > 0 && dt >= cfg.heartbeat_sec) {
            s.last_value = v;
            s.last_sent_timestamp = t;
            return true;
        }

        float dev = ocs_absf(v - s.last_value);

        if (dev < cfg.delta) {
            return false;
        }

        s.last_value = v;
        s.last_sent_timestamp = t;
        return true;
    }
};

// ============================================================================
// HYBRID FIREWALL (nâng cao)
// ============================================================================
class HybridFirewall : public FirewallBase<HybridFirewall> {
public:
    struct Config {
        float    hard_delta;
        float    threshold;
        float    alpha;
        uint32_t heartbeat_sec;
        uint32_t min_interval_sec;
    };

    struct State {
        uint32_t version;
        float    last_value;
        float    predicted;
        uint32_t last_sent_timestamp;
        bool     initialized;
    };

    static void reset_state(State& s) {
        s.version = OCS_FIREWALL_STATE_VERSION;
        s.initialized = false;
    }

    static bool impl(float v, uint32_t t, const Config& cfg, State& s)
    {
        float alpha = (cfg.alpha < 0.0f) ? 0.0f :
                      (cfg.alpha > 1.0f) ? 1.0f : cfg.alpha;

        if (!s.initialized) {
            s.last_value = v;
            s.predicted  = v;
            s.last_sent_timestamp = t;
            s.initialized = true;
            return true;
        }

        uint32_t dt = OCS_TIME_DIFF_32(t, s.last_sent_timestamp);

        // SoD (bảo hiểm)
        float dev_real = ocs_absf(v - s.last_value);
        if (dev_real > cfg.hard_delta) {
            s.last_value = v;
            s.predicted  = v;
            s.last_sent_timestamp = t;
            return true;
        }

        // heartbeat
        if (cfg.heartbeat_sec > 0 && dt >= cfg.heartbeat_sec) {
            s.last_value = v;
            s.predicted  = v;
            s.last_sent_timestamp = t;
            return true;
        }

        // predictive
        float dev_pred = ocs_absf(v - s.predicted);

        if (dev_pred < cfg.threshold) {
            s.predicted += alpha * (v - s.predicted);
            return false;
        }

        s.last_value = v;
        s.predicted  = v;
        s.last_sent_timestamp = t;
        return true;
    }
};

}

#endif