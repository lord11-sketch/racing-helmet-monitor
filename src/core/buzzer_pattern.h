// Non-blocking buzzer patterns. Hardware-independent: update() is given the
// current time and returns the frequency the buzzer should be playing right
// now (0 = silent). Covered by test/test_buzzer_pattern.
#pragma once

#include <stdint.h>

#include "alerts.h"

namespace core {

struct ToneStep {
    uint16_t freqHz;
    uint16_t onMs;
    uint16_t offMs;   // silence after the tone, before the next step
};

struct TonePattern {
    const ToneStep* steps;
    uint8_t         count;
};

// Same notes and timings as the delay()-based patterns in the original sketch.
//   Warning : 800 Hz, one 150 ms beep
//   Danger  : 1200 Hz, two 100 ms beeps 50 ms apart
//   Critical: 2000 Hz, three 80 ms beeps 20 ms apart
// AlertLevel::None returns an empty pattern.
TonePattern alertPattern(AlertLevel level);
// Start-up chime: 1000 Hz then 1400 Hz, 80 ms each, 40 ms apart.
TonePattern startupPattern();

// Plays one TonePattern against a millisecond clock.
class ToneSequencer {
public:
    ToneSequencer();

    void start(const TonePattern& pattern, uint32_t nowMs);
    void stop();

    // Frequency to output at nowMs; 0 when silent or finished.
    uint16_t update(uint32_t nowMs);
    bool     active() const { return active_; }

private:
    TonePattern pattern_;
    uint32_t    startMs_;
    bool        active_;
};

// Decides when an alert level should (re)start its pattern.
//
// While an alert is present its pattern repeats, with at least `cooldownMs`
// between starts (as in the original). Unlike the original, an alert that is
// more severe than the one last sounded starts immediately instead of waiting
// out the cooldown.
class AlertBuzzer {
public:
    explicit AlertBuzzer(uint32_t cooldownMs);

    void playStartup(uint32_t nowMs);

    // Call every loop with the current alert level. Returns the frequency to
    // output (0 = silent).
    uint16_t update(AlertLevel level, uint32_t nowMs);

private:
    ToneSequencer sequencer_;
    uint32_t      cooldownMs_;
    uint32_t      lastStartMs_;
    AlertLevel    lastLevel_;
    bool          everStarted_;
};

}  // namespace core
