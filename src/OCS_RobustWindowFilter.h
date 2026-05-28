#ifndef OCS_ROBUST_WINDOW_FILTER_H
#define OCS_ROBUST_WINDOW_FILTER_H

#include <stdint.h>

#ifndef WINDOW_SIZE
#define WINDOW_SIZE 5
#endif

// Smoothing-only robust window filter.
//
// This is a sliding-window trimmed-mean robust filter:
// collect recent samples, sort the active window, drop the min and max when
// enough samples exist, then average the remaining core samples. During startup
// with two or fewer samples, it averages all active samples.
//
// Do not hide anomaly, smoke, or fire detection inside this smoothing filter.
// Detection must remain a separate layer so filtering, thresholds, flags, and
// telemetry contracts stay independently testable.
class OCS_RobustWindowFilter {
private:
    float   buffer[WINDOW_SIZE];
    uint8_t index;
    uint8_t count;

public:
    OCS_RobustWindowFilter() : index(0), count(0) {
        for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
            buffer[i] = 0.0f;
        }
    }

    __attribute__((always_inline))
    float process(float new_value) {
        buffer[index] = new_value;
        index = (index + 1) % WINDOW_SIZE;

        if (count < WINDOW_SIZE) {
            count++;
        }

        float sorted[WINDOW_SIZE];
        for (uint8_t i = 0; i < count; i++) {
            sorted[i] = buffer[i];
        }

        for (uint8_t i = 1; i < count; i++) {
            float key = sorted[i];
            int j = i - 1;

            while (j >= 0 && sorted[j] > key) {
                sorted[j + 1] = sorted[j];
                j--;
            }
            sorted[j + 1] = key;
        }

        uint8_t start = 0U;
        uint8_t end = count;

        if (count > 2U) {
            start = 1U;
            end = count - 1U;
        }

        float sum = 0.0f;
        for (uint8_t i = start; i < end; i++) {
            sum += sorted[i];
        }

        return sum / static_cast<float>(end - start);
    }

    void reset() {
        index = 0;
        count = 0;
    }
};

#endif // OCS_ROBUST_WINDOW_FILTER_H
