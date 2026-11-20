#include "ppg_logic.h"

namespace core {

BeatAverager::BeatAverager(float minBpm, float maxBpm) : minBpm_(minBpm), maxBpm_(maxBpm) { reset(); }

void BeatAverager::reset() {
    for (uint8_t i = 0; i < kSize; ++i) rates_[i] = 0.0f;
    next_          = 0;
    count_         = 0;
    haveReference_ = false;
    lastBeatMs_    = 0;
    lastBpm_       = 0.0f;
}

void BeatAverager::dropReference() { haveReference_ = false; }

bool BeatAverager::onBeat(uint32_t tMs) {
    if (!haveReference_) {
        haveReference_ = true;
        lastBeatMs_    = tMs;
        return false;
    }

    const uint32_t deltaMs = tMs - lastBeatMs_;   // unsigned: correct across wrap-around
    lastBeatMs_ = tMs;
    if (deltaMs == 0) return false;

    const float bpm = 60000.0f / static_cast<float>(deltaMs);
    if (!(bpm > minBpm_ && bpm < maxBpm_)) return false;

    lastBpm_      = bpm;
    rates_[next_] = bpm;
    next_         = static_cast<uint8_t>((next_ + 1) % kSize);
    if (count_ < kSize) ++count_;
    return true;
}

int BeatAverager::averageBpm() const {
    if (count_ == 0) return 0;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count_; ++i) sum += rates_[i];
    return static_cast<int>(sum / static_cast<float>(count_) + 0.5f);
}

ContactDetector::ContactDetector(uint32_t presentThreshold, uint32_t absentThreshold, uint8_t debounce)
    : presentThreshold_(presentThreshold),
      absentThreshold_(absentThreshold),
      debounce_(debounce > 0 ? debounce : 1),
      counter_(0),
      present_(false) {}

void ContactDetector::reset() {
    counter_ = 0;
    present_ = false;
}

bool ContactDetector::update(uint32_t ir) {
    const bool wants = present_ ? (ir >= absentThreshold_) : (ir >= presentThreshold_);
    if (wants != present_) {
        if (++counter_ >= debounce_) {
            present_ = wants;
            counter_ = 0;
        }
    } else {
        counter_ = 0;
    }
    return present_;
}

}  // namespace core
