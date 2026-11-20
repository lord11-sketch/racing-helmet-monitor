// GSR arousal index: how far the skin-conductance signal has moved from its
// resting baseline, scaled to 0..100. Hardware-independent; covered by
// test/test_gsr_index.
//
// This is a relative measure of electrodermal activity, not a measurement of
// stress. It has no physical unit and is not comparable between people or
// sessions.
#pragma once

namespace core {

// reading, baseline and fullScale share one unit (mV or ADC counts).
//
// riseIsArousal = true  : index = (reading - baseline) / (fullScale - baseline)
// riseIsArousal = false : index = (baseline - reading) / baseline
// scaled to percent, clamped to 0..100 and truncated to an integer. Returns 0
// when the baseline leaves no headroom in the chosen direction (the original
// divided by zero in that case).
int gsrIndex(float reading, float baseline, float fullScale, bool riseIsArousal);

// True when the reading is strictly between the two rail limits.
bool gsrInRange(float reading, float railLow, float railHigh);

// Optional slow baseline follower. With tauS <= 0 the baseline stays wherever
// set() put it; otherwise it follows the reading as a first-order low-pass.
class BaselineTracker {
public:
    explicit BaselineTracker(float tauS);

    void  set(float baseline);
    float update(float reading, float dtS);
    float value() const { return baseline_; }

private:
    float tauS_;
    float baseline_;
};

}  // namespace core
