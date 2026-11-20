// Running mean used by the start-up calibration (GSR baseline, gravity vector).
#pragma once

#include <stdint.h>

namespace core {

class MeanAccumulator {
public:
    void reset() {
        sum_   = 0.0;
        count_ = 0;
    }
    void add(float value) {
        sum_ += value;
        ++count_;
    }
    uint32_t count() const { return count_; }

    // Mean of the samples added so far, or `fallback` if there are none.
    float mean(float fallback = 0.0f) const {
        return count_ > 0 ? static_cast<float>(sum_ / count_) : fallback;
    }

private:
    double   sum_   = 0.0;
    uint32_t count_ = 0;
};

}  // namespace core
