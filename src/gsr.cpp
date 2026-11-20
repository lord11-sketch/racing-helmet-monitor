#include "gsr.h"

#include <Arduino.h>

#include "config.h"
#include "core/gsr_index.h"
#include "core/mean_accumulator.h"
#include "logger.h"

namespace gsr {

namespace {

core::BaselineTracker baseline(cfg::GSR_BASELINE_TAU_S);
core::MeanAccumulator calibrationMean;
bool                  calibrating   = false;
bool                  baselineValid = false;
uint32_t              lastSampleUs  = 0;

// Average of GSR_SAMPLES conversions, about 2 ms in total.
float readAveraged() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < cfg::GSR_SAMPLES; ++i) {
        sum += cfg::GSR_USE_MILLIVOLTS ? analogReadMilliVolts(cfg::PIN_GSR)
                                       : static_cast<uint32_t>(analogRead(cfg::PIN_GSR));
        delayMicroseconds(200);
    }
    return static_cast<float>(sum) / cfg::GSR_SAMPLES;
}

}  // namespace

bool begin(Telemetry& t) {
    analogSetPinAttenuation(cfg::PIN_GSR, ADC_11db);
    const float first = readAveraged();
    lastSampleUs = micros();
    t.gsrValue   = static_cast<int>(first);
    t.gsrOk      = false;   // not usable until a baseline exists
    return core::gsrInRange(first, cfg::GSR_RAIL_LOW, cfg::GSR_RAIL_HIGH);
}

void sample(Telemetry& t) {
    const float    reading = readAveraged();
    const uint32_t nowUs   = micros();
    const float    dtS     = static_cast<float>(nowUs - lastSampleUs) * 1e-6f;
    lastSampleUs = nowUs;

    const bool inRange = core::gsrInRange(reading, cfg::GSR_RAIL_LOW, cfg::GSR_RAIL_HIGH);
    t.gsrValue = static_cast<int>(reading);

    if (calibrating) {
        if (inRange) calibrationMean.add(reading);
        t.gsrOk    = false;
        t.gsrIndex = 0;
        return;
    }

    t.gsrOk = inRange && baselineValid;
    if (!t.gsrOk) {
        t.gsrIndex = 0;
        return;
    }
    t.gsrBaseline = baseline.update(reading, dtS);   // moves only if GSR_BASELINE_TAU_S > 0
    t.gsrIndex    = core::gsrIndex(reading, t.gsrBaseline, cfg::GSR_FULL_SCALE, cfg::GSR_RISE_IS_AROUSAL);
}

void beginCalibration() {
    calibrationMean.reset();
    calibrating = true;
}

bool finishCalibration(Telemetry& t) {
    calibrating   = false;
    baselineValid = calibrationMean.count() > 0;
    if (!baselineValid) {
        logger::info("WARN GSR calibration failed: every reading was at an ADC rail, check wiring");
        return false;
    }
    baseline.set(calibrationMean.mean());
    t.gsrBaseline = baseline.value();
    logger::info("GSR baseline: %.1f %s (%lu readings)", static_cast<double>(t.gsrBaseline),
                 cfg::GSR_USE_MILLIVOLTS ? "mV" : "counts", static_cast<unsigned long>(calibrationMean.count()));
    return true;
}

}  // namespace gsr
