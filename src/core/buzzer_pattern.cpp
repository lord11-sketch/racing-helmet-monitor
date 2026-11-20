#include "buzzer_pattern.h"

namespace core {

namespace {

const ToneStep kWarning[]  = {{800, 150, 0}};
const ToneStep kDanger[]   = {{1200, 100, 50}, {1200, 100, 0}};
const ToneStep kCritical[] = {{2000, 80, 20}, {2000, 80, 20}, {2000, 80, 0}};
const ToneStep kStartup[]  = {{1000, 80, 40}, {1400, 80, 0}};

}  // namespace

TonePattern alertPattern(AlertLevel level) {
    switch (level) {
        case AlertLevel::Warning:  return {kWarning, 1};
        case AlertLevel::Danger:   return {kDanger, 2};
        case AlertLevel::Critical: return {kCritical, 3};
        default:                   return {nullptr, 0};
    }
}

TonePattern startupPattern() { return {kStartup, 2}; }

ToneSequencer::ToneSequencer() : pattern_{nullptr, 0}, startMs_(0), active_(false) {}

void ToneSequencer::start(const TonePattern& pattern, uint32_t nowMs) {
    pattern_ = pattern;
    startMs_ = nowMs;
    active_  = pattern.steps != nullptr && pattern.count > 0;
}

void ToneSequencer::stop() { active_ = false; }

uint16_t ToneSequencer::update(uint32_t nowMs) {
    if (!active_) return 0;

    uint32_t elapsed = nowMs - startMs_;   // unsigned: correct across wrap-around
    for (uint8_t i = 0; i < pattern_.count; ++i) {
        const ToneStep& step = pattern_.steps[i];
        if (elapsed < step.onMs) return step.freqHz;
        elapsed -= step.onMs;
        if (elapsed < step.offMs) return 0;
        elapsed -= step.offMs;
    }
    active_ = false;
    return 0;
}

AlertBuzzer::AlertBuzzer(uint32_t cooldownMs)
    : cooldownMs_(cooldownMs), lastStartMs_(0), lastLevel_(AlertLevel::None), everStarted_(false) {}

void AlertBuzzer::playStartup(uint32_t nowMs) { sequencer_.start(startupPattern(), nowMs); }

uint16_t AlertBuzzer::update(AlertLevel level, uint32_t nowMs) {
    if (level != AlertLevel::None) {
        const bool cooledDown = !everStarted_ || static_cast<uint32_t>(nowMs - lastStartMs_) >= cooldownMs_;
        if (cooledDown || level > lastLevel_) {
            sequencer_.start(alertPattern(level), nowMs);
            lastStartMs_ = nowMs;
            lastLevel_   = level;
            everStarted_ = true;
        }
    }
    return sequencer_.update(nowMs);
}

}  // namespace core
