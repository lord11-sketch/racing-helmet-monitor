#include "lcd_format.h"

#include <stdarg.h>
#include <stdio.h>

namespace core {

namespace {

// printf into `out`, then pad with spaces so the line is always full width.
void formatPadded(LcdLine out, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(out, kLcdCols + 1, fmt, args);
    va_end(args);

    if (n < 0) n = 0;
    if (n > static_cast<int>(kLcdCols)) n = static_cast<int>(kLcdCols);   // output was truncated
    for (size_t i = static_cast<size_t>(n); i < kLcdCols; ++i) out[i] = ' ';
    out[kLcdCols] = '\0';
}

int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Keeps "%4.1f" at four characters: NaN and negatives become 0, large values 99.9.
float clampG(float g) {
    if (!(g > 0.0f)) return 0.0f;
    return g > 99.9f ? 99.9f : g;
}

const char* levelLabel(AlertLevel level) {
    switch (level) {
        case AlertLevel::Critical: return "CRIT";
        case AlertLevel::Danger:   return "DNGR";
        case AlertLevel::Warning:  return "WARN";
        default:                   return "OK";
    }
}

}  // namespace

void formatHeartRateLine(LcdLine out, bool sensorOk, int heartRate, int hrLow, int hrHigh) {
    if (!sensorOk) {
        formatPadded(out, "HR: no sensor");
    } else if (heartRate <= 0) {
        formatPadded(out, "HR: --- bpm");
    } else {
        const char* flag = "";
        if (heartRate > hrHigh) flag = "!HI";
        else if (heartRate < hrLow) flag = "!LO";
        formatPadded(out, "HR: %3d bpm %s", clampInt(heartRate, 0, 999), flag);
    }
}

void formatSpo2Line(LcdLine out, bool sensorOk, int spo2) {
    if (!sensorOk || spo2 <= 0) {
        formatPadded(out, "SpO2:  --- %%");
    } else {
        formatPadded(out, "SpO2:  %3d %%%s", clampInt(spo2, 0, 100), spo2 < 94 ? "!" : "");
    }
}

void formatGForceLine(LcdLine out, bool sensorOk, float gTotal, float gPeak) {
    if (!sensorOk) {
        formatPadded(out, "G: no sensor");
    } else {
        formatPadded(out, "G:%4.1f Pk:%4.1f", static_cast<double>(clampG(gTotal)),
                     static_cast<double>(clampG(gPeak)));
    }
}

void formatAlertLine(LcdLine out, bool imuOk, float gLateral, AlertLevel level, uint8_t sources) {
    char cause[4];
    size_t n = 0;
    if (level != AlertLevel::None) {
        if (sources & kSourceG) cause[n++] = 'G';
        if (sources & kSourceGsr) cause[n++] = 'S';
        if (sources & kSourceHr) cause[n++] = 'H';
    }
    cause[n] = '\0';

    if (imuOk) {
        formatPadded(out, "Lat%4.1f %-4s %s", static_cast<double>(clampG(gLateral)), levelLabel(level), cause);
    } else {
        formatPadded(out, "Lat --- %-4s %s", levelLabel(level), cause);
    }
}

void formatGsrIndexLine(LcdLine out, bool sensorOk, int index) {
    if (!sensorOk) {
        formatPadded(out, "Arousal: ---");
    } else {
        formatPadded(out, "Arousal: %3d%%", clampInt(index, 0, 100));
    }
}

void formatGsrValueLine(LcdLine out, bool millivolts, int value) {
    formatPadded(out, millivolts ? "GSR mV:  %4d" : "GSR raw: %4d", clampInt(value, 0, 9999));
}

void formatCountdownLine(LcdLine out, uint32_t remainingMs) {
    // Round up so it never shows 0 while time remains.
    const uint32_t seconds = remainingMs / 1000 + (remainingMs % 1000 != 0 ? 1 : 0);
    formatPadded(out, "Stay still  %2us", static_cast<unsigned>(seconds > 99 ? 99 : seconds));
}

void formatSensorSummaryLine(LcdLine out, bool imuOk, bool ppgOk, bool gsrOk) {
    formatPadded(out, "A:%s P:%s G:%s", imuOk ? "OK" : "--", ppgOk ? "OK" : "--", gsrOk ? "OK" : "--");
}

void formatTextLine(LcdLine out, const char* text) { formatPadded(out, "%s", text); }

}  // namespace core
