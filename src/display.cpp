#include "display.h"

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <string.h>

#include "config.h"
#include "core/lcd_format.h"

namespace display {

namespace {

using core::kLcdCols;
using core::LcdLine;

const uint8_t kRows = 2;

LiquidCrystal_I2C lcd(cfg::LCD_I2C_ADDR, kLcdCols, kRows);
bool              present = false;

// What is currently on the glass, so only changed characters are rewritten.
// Each character costs six I2C transfers through the PCF8574 backpack, so a
// full 32-character redraw would hold the bus for a noticeable time.
char shadow[kRows][kLcdCols];

uint8_t  page          = 1;       // start on heart rate / SpO2
uint32_t lastPageFlip  = 0;
bool     pagesStarted  = false;
uint32_t readyUntilMs  = 0;
bool     readyBanner   = false;

void draw(const LcdLine line0, const LcdLine line1) {
    if (!present) return;
    const char* lines[kRows] = {line0, line1};
    for (uint8_t row = 0; row < kRows; ++row) {
        int cursorAt = -1;   // column the LCD cursor is known to be at, -1 = unknown
        for (uint8_t col = 0; col < kLcdCols; ++col) {
            const char c = lines[row][col];
            if (c == shadow[row][col]) continue;
            if (cursorAt != col) lcd.setCursor(col, row);
            lcd.write(static_cast<uint8_t>(c));
            shadow[row][col] = c;
            cursorAt = col + 1;
        }
    }
}

}  // namespace

bool begin() {
    Wire.beginTransmission(cfg::LCD_I2C_ADDR);
    present = (Wire.endTransmission() == 0);
    if (!present) return false;

    lcd.init();
    lcd.backlight();
    lcd.clear();
    memset(shadow, ' ', sizeof(shadow));
    return true;
}

void message(const char* line0, const char* line1) {
    LcdLine a, b;
    core::formatTextLine(a, line0);
    core::formatTextLine(b, line1);
    draw(a, b);
}

void showReady(uint32_t nowMs) {
    readyBanner  = true;
    readyUntilMs = nowMs + cfg::READY_BANNER_MS;
}

void update(const Telemetry& t, uint32_t nowMs) {
    LcdLine line0, line1;

    if (t.calibrating) {
        core::formatTextLine(line0, "Calibrating...");
        core::formatCountdownLine(line1, t.calibrationRemainingMs);
        pagesStarted = false;
        draw(line0, line1);
        return;
    }

    if (readyBanner) {
        if (static_cast<int32_t>(nowMs - readyUntilMs) < 0) {
            core::formatTextLine(line0, "System Ready!");
            core::formatSensorSummaryLine(line1, t.imuOk, t.ppgOk, t.gsrOk);
            draw(line0, line1);
            return;
        }
        readyBanner = false;
    }

    // Start the rotation timer when the pages first appear, so the first page
    // gets its full time on screen.
    if (!pagesStarted) {
        pagesStarted = true;
        page         = 1;
        lastPageFlip = nowMs;
    }

    const bool locked = t.alert >= core::AlertLevel::Danger;
    if (locked) {
        page         = 0;
        lastPageFlip = nowMs;   // resume normal rotation from the moment the lock releases
    } else if (nowMs - lastPageFlip >= cfg::LCD_PAGE_MS) {
        page         = static_cast<uint8_t>((page + 1) % 3);
        lastPageFlip = nowMs;
    }

    switch (page) {
        case 1:
            core::formatHeartRateLine(line0, t.ppgOk, t.heartRate, cfg::HR_LOW, cfg::HR_HIGH);
            core::formatSpo2Line(line1, t.ppgOk, t.spo2);
            break;
        case 2:
            core::formatGsrIndexLine(line0, t.gsrOk, t.gsrIndex);
            core::formatGsrValueLine(line1, cfg::GSR_USE_MILLIVOLTS, t.gsrValue);
            break;
        default:
            core::formatGForceLine(line0, t.imuOk, t.gTotal, t.gPeakSession);
            core::formatAlertLine(line1, t.imuOk, t.gLateral, t.alert, t.alertSources);
            break;
    }
    draw(line0, line1);
}

}  // namespace display
