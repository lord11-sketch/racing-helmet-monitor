// Text for the 16x2 LCD. Each function fills `out` with exactly 16 characters
// plus a terminating NUL, whatever the input values, so the display code never
// has to pad or truncate. Hardware-independent; covered by test/test_lcd_format.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "alerts.h"

namespace core {

const size_t kLcdCols = 16;
typedef char LcdLine[kLcdCols + 1];

// "HR:  72 bpm     " / "HR: 185 bpm !HI " / "HR: --- bpm     " / "HR: no sensor   "
void formatHeartRateLine(LcdLine out, bool sensorOk, int heartRate, int hrLow, int hrHigh);

// "SpO2:   97 %    " / "SpO2:   91 %!   " (below 94) / "SpO2:  --- %    "
void formatSpo2Line(LcdLine out, bool sensorOk, int spo2);

// "G: 1.2 Pk: 3.4  " / "G: no sensor    "
void formatGForceLine(LcdLine out, bool sensorOk, float gTotal, float gPeak);

// "Lat 0.8 OK      " / "Lat 4.2 DNGR G  " / "Lat 0.1 DNGR SH "
// The trailing letters say what raised the alert: G-force, S (GSR), Heart rate.
void formatAlertLine(LcdLine out, bool imuOk, float gLateral, AlertLevel level, uint8_t sources);

// "Arousal:  42%   " / "Arousal: ---    "
void formatGsrIndexLine(LcdLine out, bool sensorOk, int index);

// "GSR mV:  1650   " or "GSR raw: 2048   "
void formatGsrValueLine(LcdLine out, bool millivolts, int value);

// "Stay still   5s " style countdown used during calibration.
void formatCountdownLine(LcdLine out, uint32_t remainingMs);

// "A:OK P:OK G:--  " sensor summary shown once calibration finishes.
void formatSensorSummaryLine(LcdLine out, bool imuOk, bool ppgOk, bool gsrOk);

// Copies `text` into `out`, space-padded or truncated to 16 characters.
void formatTextLine(LcdLine out, const char* text);

}  // namespace core
