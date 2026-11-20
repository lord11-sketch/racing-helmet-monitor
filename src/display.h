// 16x2 I2C character LCD: boot messages, calibration screen and three rotating
// data pages.
//
//   page 1 (shown first)  heart rate / SpO2
//   page 2                GSR arousal index / GSR reading
//   page 0                g-force and session peak / lateral g and alert state
//
// Pages rotate every cfg::LCD_PAGE_MS. While the alert level is DANGER or
// CRITICAL the display stays on page 0.
#pragma once

#include <stdint.h>

#include "telemetry.h"

namespace display {

// Probes for the LCD and initialises it. Returns false if nothing answers at
// cfg::LCD_I2C_ADDR; all later calls then do nothing, so a missing display
// cannot stall the I2C bus.
bool begin();

// Two fixed lines, for boot-time messages.
void message(const char* line0, const char* line1);

// Shows the "System Ready" banner for cfg::READY_BANNER_MS, then the pages.
void showReady(uint32_t nowMs);

// Redraws from the telemetry. Only characters that changed are sent.
void update(const Telemetry& t, uint32_t nowMs);

}  // namespace display
