// Alert decision logic. Hardware-independent: no Arduino calls, no globals,
// time is passed in. Covered by test/test_alerts.
#pragma once

#include <stdint.h>

namespace core {

enum class AlertLevel : uint8_t { None = 0, Warning = 1, Danger = 2, Critical = 3 };

// Bit flags saying which measurement is currently at Warning or above.
enum AlertSource : uint8_t {
    kSourceG   = 1 << 0,
    kSourceGsr = 1 << 1,
    kSourceHr  = 1 << 2,
};

struct AlertThresholds {
    float gWarn;
    float gDanger;
    float gCritical;
    int   gsrWarn;
    int   gsrDanger;
    int   hrLow;           // alert when 0 < HR < hrLow
    int   hrHigh;          // alert when HR > hrHigh
    float gHysteresis;     // 0 = none
    int   gsrHysteresis;   // 0 = none
    int   hrHysteresis;    // 0 = none
};

struct AlertInputs {
    float gDynamic;    // gravity-removed acceleration magnitude, g
    int   gsrIndex;    // 0..100
    int   heartRate;   // bpm; 0 = no reading (never alerts)
};

struct AlertResult {
    AlertLevel level;
    uint8_t    sources;   // AlertSource bits
};

// Turns measurements into an alert level.
//
//   g-force     >= gWarn -> Warning, >= gDanger -> Danger, >= gCritical -> Critical
//   GSR index   >= gsrWarn -> Warning, >= gsrDanger -> Danger
//   heart rate  > hrHigh, or 0 < HR < hrLow -> Danger
//
// The result is the highest of the three. With every hysteresis value at zero
// this is exactly the rule in the original sketch. With hysteresis, a level is
// only released once the value has dropped that far below its threshold (or,
// for a low heart rate, risen that far above it).
class AlertEvaluator {
public:
    explicit AlertEvaluator(const AlertThresholds& thresholds);

    AlertResult update(const AlertInputs& in);
    void reset();

private:
    enum class HrState : uint8_t { Normal, Low, High };

    AlertThresholds th_;
    uint8_t         gLevel_;
    uint8_t         gsrLevel_;
    HrState         hrState_;
};

// Keeps an alert visible after its cause has gone.
//
// The output is the instantaneous level, except that it may not drop until
// `holdMs` after the last moment the instantaneous level was at least as high.
// Optionally a Critical alert stays until acknowledge() is called.
// holdMs = 0 with latchCritical = false passes the input straight through.
class AlertLatch {
public:
    AlertLatch(uint32_t holdMs, bool latchCritical);

    AlertResult update(const AlertResult& instant, uint32_t nowMs);

    // Clears a held or latched alert. If the cause is still present the alert
    // comes straight back on the next update.
    void acknowledge();

private:
    uint32_t    holdMs_;
    bool        latchCritical_;
    AlertResult held_;
    uint32_t    sinceMs_;
};

}  // namespace core
