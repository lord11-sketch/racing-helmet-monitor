#include "logger.h"

#include <Arduino.h>
#include <stdarg.h>

#include "config.h"

namespace logger {

namespace {

// Bits of the `status` column.
enum StatusBit : uint8_t {
    kStatusImuOk        = 1 << 0,
    kStatusPpgOk        = 1 << 1,
    kStatusGsrOk        = 1 << 2,
    kStatusPpgContact   = 1 << 3,
    kStatusImuSaturated = 1 << 4,
    kStatusCalibrating  = 1 << 5,
};

uint8_t statusBits(const Telemetry& t) {
    uint8_t s = 0;
    if (t.imuOk) s |= kStatusImuOk;
    if (t.ppgOk) s |= kStatusPpgOk;
    if (t.gsrOk) s |= kStatusGsrOk;
    if (t.ppgContact) s |= kStatusPpgContact;
    if (t.imuSaturated) s |= kStatusImuSaturated;
    if (t.calibrating) s |= kStatusCalibrating;
    return s;
}

}  // namespace

void begin() { Serial.begin(cfg::SERIAL_BAUD); }

void info(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print("# ");
    Serial.println(buf);
}

void printHeader() {
    // The first ten columns keep the order of the original sketch's CSV.
    Serial.print("time_ms,g_total,g_lat,g_long,g_peak_session,gsr_index,");
    Serial.print(cfg::GSR_USE_MILLIVOLTS ? "gsr_mv" : "gsr_raw");
    Serial.println(",hr_bpm,spo2_pct,alert,g_peak_window,g_vert,hr_maxim_bpm,alert_instant,alert_sources,status");
}

void printHelp() {
    info("commands: a = acknowledge alert, r = reset session peak, c = recalibrate, h = help");
}

void logRow(Telemetry& t, uint32_t nowMs) {
    char row[160];
    snprintf(row, sizeof(row), "%lu,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%.2f,%.2f,%d,%d,%u,%u",
             static_cast<unsigned long>(nowMs),
             static_cast<double>(t.gTotal),
             static_cast<double>(t.gLateral),
             static_cast<double>(t.gLong),
             static_cast<double>(t.gPeakSession),
             t.gsrIndex,
             t.gsrValue,
             t.heartRate,
             t.spo2,
             static_cast<int>(t.alert),
             static_cast<double>(t.gPeakLog),
             static_cast<double>(t.gVertical),
             t.hrMaxim,
             static_cast<int>(t.alertInstant),
             static_cast<unsigned>(t.alertSources),
             static_cast<unsigned>(statusBits(t)));
    Serial.println(row);

    t.gPeakLog     = 0.0f;
    t.imuSaturated = false;
}

Command pollCommand() {
    while (Serial.available() > 0) {
        switch (Serial.read()) {
            case 'a': case 'A': return Command::Acknowledge;
            case 'r': case 'R': return Command::ResetPeak;
            case 'c': case 'C': return Command::Recalibrate;
            case 'h': case 'H': case '?': return Command::Help;
            default: break;   // ignore line endings and anything else
        }
    }
    return Command::None;
}

}  // namespace logger
