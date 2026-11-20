#include "alerts.h"

namespace core {

namespace {

// Level of a value against ascending thresholds up[0..n-1], with hysteresis on
// the way down: level i+1 is entered at v >= up[i] and left at v < up[i] - hyst.
uint8_t stepLevel(uint8_t previous, float v, const float* up, uint8_t n, float hyst) {
    if (!(v == v)) v = 0.0f;   // NaN never raises or holds an alert

    uint8_t raw = 0;
    for (uint8_t i = 0; i < n; ++i) {
        if (v >= up[i]) raw = static_cast<uint8_t>(i + 1);
    }
    if (raw >= previous) return raw;

    uint8_t level = previous;
    while (level > raw && v < up[level - 1] - hyst) --level;
    return level;
}

}  // namespace

AlertEvaluator::AlertEvaluator(const AlertThresholds& thresholds)
    : th_(thresholds), gLevel_(0), gsrLevel_(0), hrState_(HrState::Normal) {}

void AlertEvaluator::reset() {
    gLevel_   = 0;
    gsrLevel_ = 0;
    hrState_  = HrState::Normal;
}

AlertResult AlertEvaluator::update(const AlertInputs& in) {
    const float gUp[3]   = {th_.gWarn, th_.gDanger, th_.gCritical};
    const float gsrUp[2] = {static_cast<float>(th_.gsrWarn), static_cast<float>(th_.gsrDanger)};

    gLevel_   = stepLevel(gLevel_, in.gDynamic, gUp, 3, th_.gHysteresis);
    gsrLevel_ = stepLevel(gsrLevel_, static_cast<float>(in.gsrIndex), gsrUp, 2,
                          static_cast<float>(th_.gsrHysteresis));

    const int hr = in.heartRate;
    if (hr <= 0) {
        hrState_ = HrState::Normal;   // no reading: nothing to alert on
    } else if (hr > th_.hrHigh) {
        hrState_ = HrState::High;
    } else if (hr < th_.hrLow) {
        hrState_ = HrState::Low;
    } else if (hrState_ == HrState::High && hr > th_.hrHigh - th_.hrHysteresis) {
        // still inside the hysteresis band below the high limit
    } else if (hrState_ == HrState::Low && hr < th_.hrLow + th_.hrHysteresis) {
        // still inside the hysteresis band above the low limit
    } else {
        hrState_ = HrState::Normal;
    }
    const uint8_t hrLevel =
        (hrState_ == HrState::Normal) ? 0 : static_cast<uint8_t>(AlertLevel::Danger);

    uint8_t level = gLevel_;
    if (gsrLevel_ > level) level = gsrLevel_;
    if (hrLevel > level) level = hrLevel;

    AlertResult result;
    result.level   = static_cast<AlertLevel>(level);
    result.sources = 0;
    if (gLevel_ > 0) result.sources |= kSourceG;
    if (gsrLevel_ > 0) result.sources |= kSourceGsr;
    if (hrLevel > 0) result.sources |= kSourceHr;
    return result;
}

AlertLatch::AlertLatch(uint32_t holdMs, bool latchCritical)
    : holdMs_(holdMs), latchCritical_(latchCritical), held_{AlertLevel::None, 0}, sinceMs_(0) {}

AlertResult AlertLatch::update(const AlertResult& instant, uint32_t nowMs) {
    // Same or higher level than the one being held: follow it, restart the hold.
    if (instant.level != AlertLevel::None && instant.level >= held_.level) {
        held_    = instant;
        sinceMs_ = nowMs;
        return held_;
    }
    if (held_.level == AlertLevel::None) return instant;

    // The cause has dropped below the held level.
    const bool latched = latchCritical_ && held_.level == AlertLevel::Critical;
    if (!latched && static_cast<uint32_t>(nowMs - sinceMs_) >= holdMs_) {
        held_    = instant;
        sinceMs_ = nowMs;
    }
    return held_;
}

void AlertLatch::acknowledge() {
    held_.level   = AlertLevel::None;
    held_.sources = 0;
}

}  // namespace core
