#include "ppg.h"

#include <Arduino.h>
#include <MAX30105.h>
#include <Wire.h>
#include <heartRate.h>
#include <spo2_algorithm.h>
#include <string.h>

#include "config.h"
#include "core/ppg_logic.h"

// The Maxim algorithm hard-codes its sample rate and window length. If the
// sensor configuration in config.h ever drifts from them, fail the build
// rather than produce plausible-looking wrong numbers.
static_assert(cfg::PPG_OUTPUT_HZ == FreqS, "PPG output rate must equal FreqS in spo2_algorithm.h");
static_assert(cfg::PPG_SPO2_WINDOW == BUFFER_SIZE, "SpO2 window must equal BUFFER_SIZE in spo2_algorithm.h");

namespace ppg {

namespace {

MAX30105 sensor;
bool     present = false;

core::BeatAverager    beats(cfg::PPG_BPM_MIN, cfg::PPG_BPM_MAX);
core::ContactDetector contact(cfg::PPG_IR_PRESENT, cfg::PPG_IR_ABSENT, cfg::PPG_CONTACT_DEBOUNCE);

// Rolling window for the SpO2 algorithm, oldest sample first.
uint32_t irWindow[cfg::PPG_SPO2_WINDOW];
uint32_t redWindow[cfg::PPG_SPO2_WINDOW];
uint8_t  windowFill      = 0;
uint8_t  samplesSinceRun = 0;

// Time is counted in samples, not millis(): every sample is exactly
// 1/PPG_OUTPUT_HZ after the previous one by the sensor's own clock, however
// late the main loop gets round to reading it.
uint32_t sampleCount     = 0;
uint32_t lastBeatMs      = 0;   // sample-clock time of the last accepted beat
uint32_t lastSpo2Ms      = 0;   // sample-clock time of the last valid SpO2
uint32_t lastSampleWall  = 0;   // millis() when a sample last arrived

// The driver keeps only this many samples between check() and nextSample().
const uint16_t kDriverStorage = STORAGE_SIZE;

uint32_t sampleClockMs() {
    return static_cast<uint32_t>(static_cast<uint64_t>(sampleCount) * 1000u / cfg::PPG_OUTPUT_HZ);
}

void clearReadings(Telemetry& t) {
    t.heartRate = 0;
    t.spo2      = 0;
    t.hrMaxim   = 0;
}

void resetSignalState(Telemetry& t) {
    beats.reset();
    windowFill      = 0;
    samplesSinceRun = 0;
    clearReadings(t);
}

void runSpo2(Telemetry& t, uint32_t tMs) {
    int32_t spo2 = 0, hr = 0;
    int8_t  spo2Valid = 0, hrValid = 0;
    maxim_heart_rate_and_oxygen_saturation(irWindow, cfg::PPG_SPO2_WINDOW, redWindow, &spo2, &spo2Valid, &hr,
                                           &hrValid);

    if (spo2Valid && spo2 > 50 && spo2 <= 100) {
        t.spo2     = static_cast<int>(spo2);
        lastSpo2Ms = tMs;
    }
    // The algorithm's own heart-rate estimate is logged for comparison only;
    // the displayed and alerting heart rate always comes from beat timing.
    t.hrMaxim = (hrValid && hr > cfg::PPG_BPM_MIN && hr < cfg::PPG_BPM_MAX) ? static_cast<int>(hr) : 0;
}

void processSample(Telemetry& t, uint32_t ir, uint32_t red) {
    ++sampleCount;
    const uint32_t tMs = sampleClockMs();

    // Contact first: a sample taken with no skin contact must not reach the
    // beat detector or the SpO2 window.
    const bool hadContact = t.ppgContact;
    t.ppgContact = contact.update(ir);
    if (!t.ppgContact) {
        if (hadContact) resetSignalState(t);
        return;
    }
    if (!hadContact) {
        lastBeatMs = tMs;
        lastSpo2Ms = tMs;
    }

    if (checkForBeat(static_cast<int32_t>(ir)) && beats.onBeat(tMs)) {
        t.heartRate = beats.averageBpm();
        lastBeatMs  = tMs;
    }

    if (windowFill < cfg::PPG_SPO2_WINDOW) {
        irWindow[windowFill]  = ir;
        redWindow[windowFill] = red;
        ++windowFill;
    } else {
        memmove(irWindow, irWindow + 1, sizeof(irWindow) - sizeof(irWindow[0]));
        memmove(redWindow, redWindow + 1, sizeof(redWindow) - sizeof(redWindow[0]));
        irWindow[cfg::PPG_SPO2_WINDOW - 1]  = ir;
        redWindow[cfg::PPG_SPO2_WINDOW - 1] = red;
    }
    if (samplesSinceRun < 255) ++samplesSinceRun;
    if (windowFill == cfg::PPG_SPO2_WINDOW && samplesSinceRun >= cfg::PPG_SPO2_STEP) {
        samplesSinceRun = 0;
        runSpo2(t, tMs);
    }

    // Blank readings that have stopped updating rather than show them as live.
    if (cfg::PPG_HR_STALE_MS > 0 && t.heartRate > 0 && tMs - lastBeatMs > cfg::PPG_HR_STALE_MS) {
        t.heartRate = 0;
        beats.reset();
    }
    if (cfg::PPG_SPO2_STALE_MS > 0 && t.spo2 > 0 && tMs - lastSpo2Ms > cfg::PPG_SPO2_STALE_MS) {
        t.spo2 = 0;
    }
}

}  // namespace

bool begin(Telemetry& t) {
    present = sensor.begin(Wire, cfg::I2C_CLOCK_HZ);
    if (present) {
        // Red + IR mode. The LED amplitude is a register value (0.2 mA per
        // step), not milliamps.
        sensor.setup(cfg::PPG_LED_AMPLITUDE, cfg::PPG_SAMPLE_AVERAGE, 2, cfg::PPG_SAMPLE_RATE_HZ,
                     cfg::PPG_PULSE_WIDTH_US, cfg::PPG_ADC_RANGE);
        lastSampleWall = millis();
    }
    t.ppgOk = present;
    return present;
}

void poll(Telemetry& t, uint32_t nowMs) {
    if (!present) return;

    const uint16_t fresh = sensor.check();

    if (fresh >= kDriverStorage) {
        // More samples arrived than the driver can hold, so some were
        // overwritten. Throw the remainder away, keep the sample clock honest,
        // and make the beat timer start again from the next beat.
        while (sensor.available()) sensor.nextSample();
        sampleCount += fresh;
        beats.dropReference();
        windowFill      = 0;
        samplesSinceRun = 0;
        lastSampleWall  = nowMs;
        return;
    }

    while (sensor.available()) {
        // getFIFOIR()/getFIFORed() return the oldest unread sample without
        // touching the bus. (getIR()/getRed() would each block until a *new*
        // sample arrives.)
        const uint32_t ir  = sensor.getFIFOIR();
        const uint32_t red = sensor.getFIFORed();
        sensor.nextSample();
        processSample(t, ir, red);
        lastSampleWall = nowMs;
    }

    // No data at all for a while: the sensor has stopped responding.
    const bool stalled = (nowMs - lastSampleWall) > cfg::PPG_STALL_MS;
    if (stalled && t.ppgOk) {
        t.ppgContact = false;
        contact.reset();
        resetSignalState(t);
    }
    t.ppgOk = !stalled;
}

}  // namespace ppg
