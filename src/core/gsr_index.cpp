#include "gsr_index.h"

namespace core {

int gsrIndex(float reading, float baseline, float fullScale, bool riseIsArousal) {
    const float span  = riseIsArousal ? (fullScale - baseline) : baseline;
    const float delta = riseIsArousal ? (reading - baseline) : (baseline - reading);

    // Require at least 1 % of full scale as headroom; less than that and the
    // index would be noise divided by nearly nothing.
    if (!(span >= 0.01f * fullScale)) return 0;

    float percent = delta / span * 100.0f;
    if (!(percent > 0.0f)) percent = 0.0f;   // also catches NaN
    if (percent > 100.0f) percent = 100.0f;
    return static_cast<int>(percent);
}

bool gsrInRange(float reading, float railLow, float railHigh) {
    return reading > railLow && reading < railHigh;
}

BaselineTracker::BaselineTracker(float tauS) : tauS_(tauS), baseline_(0.0f) {}

void BaselineTracker::set(float baseline) { baseline_ = baseline; }

float BaselineTracker::update(float reading, float dtS) {
    if (tauS_ > 0.0f && dtS > 0.0f) {
        const float alpha = dtS / (tauS_ + dtS);
        baseline_ += (reading - baseline_) * alpha;
    }
    return baseline_;
}

}  // namespace core
