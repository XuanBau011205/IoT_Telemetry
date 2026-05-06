#ifndef OCS_MEDIAN_H
#define OCS_MEDIAN_H

#include <stdint.h>

// Cấu hình kích thước cửa sổ
#ifndef WINDOW_SIZE
#define WINDOW_SIZE 5
#endif

class OcsMedianFilter {
private:
    float   buffer[WINDOW_SIZE];
    uint8_t index;
    uint8_t count;

public:
    OcsMedianFilter() : index(0), count(0) {
        for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
            buffer[i] = 0.0f;
        }
    }

    __attribute__((always_inline))
    float process(float new_value) {
        // 1. Push vào ring buffer
        buffer[index] = new_value;
        index = (index + 1) % WINDOW_SIZE;

        if (count < WINDOW_SIZE) count++;

        // 2. Copy sang mảng tạm
        float sorted[WINDOW_SIZE];
        for (uint8_t i = 0; i < count; i++) {
            sorted[i] = buffer[i];
        }

        // 3. Insertion sort (tối ưu cho N nhỏ)
        for (uint8_t i = 1; i < count; i++) {
            float key = sorted[i];
            int j = i - 1;

            while (j >= 0 && sorted[j] > key) {
                sorted[j + 1] = sorted[j];
                j--;
            }
            sorted[j + 1] = key;
        }

        // 4. Lấy median
        if (count % 2 == 0) {
            return (sorted[count/2 - 1] + sorted[count/2]) * 0.5f;
        } else {
            return sorted[count/2];
        }
    }

    void reset() {
        index = 0;
        count = 0;
    }
};

#endif // OCS_MEDIAN_H