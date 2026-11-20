// The one shared data record: sensors write into it, the alert logic reads it,
// the display and logger print it. Plain data, no behaviour.
#pragma once

#include <stdint.h>

#include "core/alerts.h"

struct Telemetry {
    // ── Accelerometer ────────────────────────────────────────────────────────
    bool  imuOk        = false;   // found at start-up and still responding
    float gTotal       = 0.0f;    // gravity-removed acceleration magnitude, latest sample (g)
    float gLateral     = 0.0f;    // |lateral component| (g)
    float gLong        = 0.0f;    // |longitudinal component| (g)
    float gVertical    = 0.0f;    // signed vertical component (g)
    float gPeakSession = 0.0f;    // largest gTotal since start-up or the last reset
    float gPeakLog     = 0.0f;    // largest gTotal since the previous log row
    bool  imuSaturated = false;   // an axis clipped since the previous log row

    // ── Pulse oximeter ───────────────────────────────────────────────────────
    bool ppgOk       = false;   // found at start-up and still delivering samples
    bool ppgContact  = false;   // skin contact detected
    int  heartRate   = 0;       // bpm from beat-to-beat timing; 0 = no reading
    int  spo2        = 0;       // percent; 0 = no reading
    int  hrMaxim     = 0;       // bpm from the Maxim window algorithm (diagnostic only)

    // ── GSR ──────────────────────────────────────────────────────────────────
    bool  gsrOk       = false;   // reading is off the ADC rails and a baseline exists
    int   gsrIndex    = 0;       // 0..100 arousal index
    int   gsrValue    = 0;       // averaged reading, mV or ADC counts (see config.h)
    float gsrBaseline = 0.0f;    // same unit as gsrValue

    // ── Alerts ───────────────────────────────────────────────────────────────
    core::AlertLevel alert        = core::AlertLevel::None;   // after hold/latch: shown, logged, sounded
    core::AlertLevel alertInstant = core::AlertLevel::None;   // before hold/latch
    uint8_t          alertSources = 0;                        // core::AlertSource bits for `alert`

    // ── System ───────────────────────────────────────────────────────────────
    bool     calibrating            = false;
    uint32_t calibrationRemainingMs = 0;
};
