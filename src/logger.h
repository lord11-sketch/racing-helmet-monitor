// Serial output (CSV telemetry plus '#'-prefixed status lines) and the small
// set of single-character serial commands.
#pragma once

#include <stdint.h>

#include "telemetry.h"

namespace logger {

enum class Command : uint8_t {
    None,
    Acknowledge,   // 'a'  clear a held or latched alert
    ResetPeak,     // 'r'  reset the session peak g
    Recalibrate,   // 'c'  rerun the 5 s calibration
    Help,          // 'h' or '?'
};

void begin();

// printf-style status line. Written as "# text" so anything parsing the CSV
// can skip it.
void info(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Prints the CSV column names (as a normal, un-prefixed line).
void printHeader();
void printHelp();

// Writes one CSV row. Clears the per-row accumulators (gPeakLog, imuSaturated).
void logRow(Telemetry& t, uint32_t nowMs);

// Non-blocking: returns the next command typed on the serial port, if any.
Command pollCommand();

}  // namespace logger
