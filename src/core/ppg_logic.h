// Heart-rate bookkeeping that sits around the beat detector.
// Hardware-independent; covered by test/test_ppg_logic.
#pragma once

#include <stdint.h>

namespace core {

// Turns a stream of beat timestamps into a short moving-average heart rate.
//
// Differences from the averaging in the original sketch:
//  - the average is taken over the beats actually stored, so the first few
//    readings are not diluted by empty slots;
//  - the first beat after a reset only sets the time reference, instead of
//    producing an interval measured from an unrelated moment;
//  - rates are kept as float, not truncated to a byte.
class BeatAverager {
public:
    static const uint8_t kSize = 4;

    // Beat-to-beat rates outside the open interval (minBpm, maxBpm) are ignored.
    BeatAverager(float minBpm, float maxBpm);

    // Forget everything (use when skin contact is lost).
    void reset();

    // Forget only the time of the previous beat (use after a gap in the sample
    // stream). The next beat becomes a new reference; the average is kept.
    void dropReference();

    // Report a detected beat at time tMs (any monotonic millisecond clock;
    // wrap-around is handled). Returns true if the average was updated.
    bool onBeat(uint32_t tMs);

    // Rounded mean of the stored rates, or 0 if there are none yet.
    int averageBpm() const;

    float   lastBpm() const { return lastBpm_; }
    uint8_t count() const { return count_; }

private:
    float    minBpm_;
    float    maxBpm_;
    float    rates_[kSize];
    uint8_t  next_;
    uint8_t  count_;
    bool     haveReference_;
    uint32_t lastBeatMs_;
    float    lastBpm_;
};

// Skin-contact detection from the raw IR level, with hysteresis and debounce.
// Contact is declared after `debounce` consecutive samples >= presentThreshold
// and withdrawn after `debounce` consecutive samples < absentThreshold.
class ContactDetector {
public:
    ContactDetector(uint32_t presentThreshold, uint32_t absentThreshold, uint8_t debounce);

    // Feed one IR sample; returns the (possibly updated) contact state.
    bool update(uint32_t ir);
    bool present() const { return present_; }
    void reset();

private:
    uint32_t presentThreshold_;
    uint32_t absentThreshold_;
    uint8_t  debounce_;
    uint8_t  counter_;
    bool     present_;
};

}  // namespace core
