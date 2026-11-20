// Unit tests for the LCD line formatting (src/core/lcd_format.*).
// Every line must be exactly 16 characters whatever the inputs.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "core/lcd_format.h"

using core::AlertLevel;
using core::LcdLine;

namespace {

void assertFullWidth(const char* line) { TEST_ASSERT_EQUAL_UINT32(16, strlen(line)); }

}  // namespace

void setUp() {}
void tearDown() {}

void test_heart_rate_line() {
    LcdLine out;
    core::formatHeartRateLine(out, true, 72, 40, 180);
    TEST_ASSERT_EQUAL_STRING("HR:  72 bpm     ", out);
    core::formatHeartRateLine(out, true, 185, 40, 180);
    TEST_ASSERT_EQUAL_STRING("HR: 185 bpm !HI ", out);
    core::formatHeartRateLine(out, true, 35, 40, 180);
    TEST_ASSERT_EQUAL_STRING("HR:  35 bpm !LO ", out);
    core::formatHeartRateLine(out, true, 0, 40, 180);
    TEST_ASSERT_EQUAL_STRING("HR: --- bpm     ", out);
    core::formatHeartRateLine(out, false, 72, 40, 180);
    TEST_ASSERT_EQUAL_STRING("HR: no sensor   ", out);
}

// The original's "no reading" SpO2 line had a lone '%' in its printf format.
void test_spo2_line() {
    LcdLine out;
    core::formatSpo2Line(out, true, 97);
    TEST_ASSERT_EQUAL_STRING("SpO2:   97 %    ", out);
    core::formatSpo2Line(out, true, 91);
    TEST_ASSERT_EQUAL_STRING("SpO2:   91 %!   ", out);
    core::formatSpo2Line(out, true, 0);
    TEST_ASSERT_EQUAL_STRING("SpO2:  --- %    ", out);
    core::formatSpo2Line(out, false, 97);
    TEST_ASSERT_EQUAL_STRING("SpO2:  --- %    ", out);
}

void test_g_force_line() {
    LcdLine out;
    core::formatGForceLine(out, true, 1.2f, 3.4f);
    TEST_ASSERT_EQUAL_STRING("G: 1.2 Pk: 3.4  ", out);
    core::formatGForceLine(out, false, 1.2f, 3.4f);
    TEST_ASSERT_EQUAL_STRING("G: no sensor    ", out);
}

void test_alert_line_shows_level_and_cause() {
    LcdLine out;
    core::formatAlertLine(out, true, 0.8f, AlertLevel::None, 0);
    TEST_ASSERT_EQUAL_STRING("Lat 0.8 OK      ", out);
    core::formatAlertLine(out, true, 4.2f, AlertLevel::Danger, core::kSourceG);
    TEST_ASSERT_EQUAL_STRING("Lat 4.2 DNGR G  ", out);
    core::formatAlertLine(out, true, 0.1f, AlertLevel::Warning, core::kSourceGsr);
    TEST_ASSERT_EQUAL_STRING("Lat 0.1 WARN S  ", out);
    core::formatAlertLine(out, true, 7.0f, AlertLevel::Critical,
                          core::kSourceG | core::kSourceGsr | core::kSourceHr);
    TEST_ASSERT_EQUAL_STRING("Lat 7.0 CRIT GSH", out);
    core::formatAlertLine(out, false, 0.0f, AlertLevel::Danger, core::kSourceHr);
    TEST_ASSERT_EQUAL_STRING("Lat --- DNGR H  ", out);
}

void test_gsr_lines() {
    LcdLine out;
    core::formatGsrIndexLine(out, true, 42);
    TEST_ASSERT_EQUAL_STRING("Arousal:  42%   ", out);
    core::formatGsrIndexLine(out, false, 42);
    TEST_ASSERT_EQUAL_STRING("Arousal: ---    ", out);
    core::formatGsrValueLine(out, true, 1650);
    TEST_ASSERT_EQUAL_STRING("GSR mV:  1650   ", out);
    core::formatGsrValueLine(out, false, 2048);
    TEST_ASSERT_EQUAL_STRING("GSR raw: 2048   ", out);
}

void test_countdown_rounds_up() {
    LcdLine out;
    core::formatCountdownLine(out, 5000);
    TEST_ASSERT_EQUAL_STRING("Stay still   5s ", out);
    core::formatCountdownLine(out, 4001);
    TEST_ASSERT_EQUAL_STRING("Stay still   5s ", out);
    core::formatCountdownLine(out, 1);
    TEST_ASSERT_EQUAL_STRING("Stay still   1s ", out);
    core::formatCountdownLine(out, 0);
    TEST_ASSERT_EQUAL_STRING("Stay still   0s ", out);
}

void test_sensor_summary_line() {
    LcdLine out;
    core::formatSensorSummaryLine(out, true, false, true);
    TEST_ASSERT_EQUAL_STRING("A:OK P:-- G:OK  ", out);
}

void test_text_line_pads_and_truncates() {
    LcdLine out;
    core::formatTextLine(out, "Hi");
    TEST_ASSERT_EQUAL_STRING("Hi              ", out);
    core::formatTextLine(out, "0123456789ABCDEFGHIJ");
    TEST_ASSERT_EQUAL_STRING("0123456789ABCDEF", out);
    core::formatTextLine(out, "");
    assertFullWidth(out);
}

void test_every_line_is_full_width_for_extreme_values() {
    LcdLine out;

    const int ints[] = {-100000, -1, 0, 1, 99, 100, 999, 1000, 100000};
    for (size_t i = 0; i < sizeof(ints) / sizeof(ints[0]); ++i) {
        core::formatHeartRateLine(out, true, ints[i], 40, 180);
        assertFullWidth(out);
        core::formatSpo2Line(out, true, ints[i]);
        assertFullWidth(out);
        core::formatGsrIndexLine(out, true, ints[i]);
        assertFullWidth(out);
        core::formatGsrValueLine(out, true, ints[i]);
        assertFullWidth(out);
        core::formatGsrValueLine(out, false, ints[i]);
        assertFullWidth(out);
    }

    const float floats[] = {-5.0f, 0.0f, 0.04f, 9.96f, 13.9f, 99.99f, 1.0e9f, NAN, INFINITY};
    for (size_t i = 0; i < sizeof(floats) / sizeof(floats[0]); ++i) {
        core::formatGForceLine(out, true, floats[i], floats[i]);
        assertFullWidth(out);
        core::formatAlertLine(out, true, floats[i], AlertLevel::Critical, 0xFF);
        assertFullWidth(out);
    }

    core::formatCountdownLine(out, 0xFFFFFFFFu);
    assertFullWidth(out);
}

int runTests() {
    UNITY_BEGIN();
    RUN_TEST(test_heart_rate_line);
    RUN_TEST(test_spo2_line);
    RUN_TEST(test_g_force_line);
    RUN_TEST(test_alert_line_shows_level_and_cause);
    RUN_TEST(test_gsr_lines);
    RUN_TEST(test_countdown_rounds_up);
    RUN_TEST(test_sensor_summary_line);
    RUN_TEST(test_text_line_pads_and_truncates);
    RUN_TEST(test_every_line_is_full_width_for_extreme_values);
    return UNITY_END();
}

#ifdef ARDUINO
#include <Arduino.h>
void setup() {
    delay(2000);   // give the test runner time to connect
    runTests();
}
void loop() {}
#else
int main() { return runTests(); }
#endif
