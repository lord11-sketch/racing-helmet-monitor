// Unit tests for the non-blocking buzzer patterns (src/core/buzzer_pattern.*).
#include <unity.h>

#include "core/buzzer_pattern.h"

using core::AlertBuzzer;
using core::AlertLevel;
using core::ToneSequencer;

void setUp() {}
void tearDown() {}

// ── ToneSequencer ────────────────────────────────────────────────────────────

void test_idle_sequencer_is_silent() {
    ToneSequencer s;
    TEST_ASSERT_FALSE(s.active());
    TEST_ASSERT_EQUAL_UINT16(0, s.update(123));
}

void test_warning_is_one_150ms_beep() {
    ToneSequencer s;
    s.start(core::alertPattern(AlertLevel::Warning), 1000);
    TEST_ASSERT_EQUAL_UINT16(800, s.update(1000));
    TEST_ASSERT_EQUAL_UINT16(800, s.update(1149));
    TEST_ASSERT_EQUAL_UINT16(0, s.update(1150));
    TEST_ASSERT_FALSE(s.active());
}

void test_danger_is_two_beeps_with_a_gap() {
    ToneSequencer s;
    s.start(core::alertPattern(AlertLevel::Danger), 1000);
    TEST_ASSERT_EQUAL_UINT16(1200, s.update(1000));
    TEST_ASSERT_EQUAL_UINT16(1200, s.update(1099));
    TEST_ASSERT_EQUAL_UINT16(0, s.update(1100));      // 50 ms gap
    TEST_ASSERT_EQUAL_UINT16(0, s.update(1149));
    TEST_ASSERT_TRUE(s.active());
    TEST_ASSERT_EQUAL_UINT16(1200, s.update(1150));   // second beep
    TEST_ASSERT_EQUAL_UINT16(1200, s.update(1249));
    TEST_ASSERT_EQUAL_UINT16(0, s.update(1250));
    TEST_ASSERT_FALSE(s.active());
}

void test_critical_is_three_beeps() {
    ToneSequencer s;
    s.start(core::alertPattern(AlertLevel::Critical), 0);
    TEST_ASSERT_EQUAL_UINT16(2000, s.update(0));
    TEST_ASSERT_EQUAL_UINT16(0, s.update(80));
    TEST_ASSERT_EQUAL_UINT16(2000, s.update(100));
    TEST_ASSERT_EQUAL_UINT16(0, s.update(180));
    TEST_ASSERT_EQUAL_UINT16(2000, s.update(200));
    TEST_ASSERT_EQUAL_UINT16(2000, s.update(279));
    TEST_ASSERT_EQUAL_UINT16(0, s.update(280));
    TEST_ASSERT_FALSE(s.active());
}

void test_startup_chime() {
    ToneSequencer s;
    s.start(core::startupPattern(), 0);
    TEST_ASSERT_EQUAL_UINT16(1000, s.update(0));
    TEST_ASSERT_EQUAL_UINT16(0, s.update(80));
    TEST_ASSERT_EQUAL_UINT16(1400, s.update(120));
    TEST_ASSERT_EQUAL_UINT16(0, s.update(200));
}

void test_no_pattern_for_no_alert() {
    ToneSequencer s;
    s.start(core::alertPattern(AlertLevel::None), 0);
    TEST_ASSERT_FALSE(s.active());
    TEST_ASSERT_EQUAL_UINT16(0, s.update(0));
}

void test_pattern_survives_clock_wraparound() {
    ToneSequencer  s;
    const uint32_t start = 0xFFFFFFF0u;   // 16 ms before the counter wraps
    s.start(core::alertPattern(AlertLevel::Warning), start);
    TEST_ASSERT_EQUAL_UINT16(800, s.update(start + 100u));   // already wrapped
    TEST_ASSERT_EQUAL_UINT16(0, s.update(start + 150u));
}

void test_stop_silences_immediately() {
    ToneSequencer s;
    s.start(core::alertPattern(AlertLevel::Warning), 0);
    s.stop();
    TEST_ASSERT_EQUAL_UINT16(0, s.update(10));
}

// ── AlertBuzzer ──────────────────────────────────────────────────────────────

void test_no_alert_no_sound() {
    AlertBuzzer b(1000);
    TEST_ASSERT_EQUAL_UINT16(0, b.update(AlertLevel::None, 0));
    TEST_ASSERT_EQUAL_UINT16(0, b.update(AlertLevel::None, 5000));
}

void test_alert_sounds_on_the_first_update() {
    AlertBuzzer b(1000);
    TEST_ASSERT_EQUAL_UINT16(800, b.update(AlertLevel::Warning, 50));
}

void test_pattern_repeats_only_after_the_cooldown() {
    AlertBuzzer b(1000);
    b.update(AlertLevel::Warning, 0);
    TEST_ASSERT_EQUAL_UINT16(0, b.update(AlertLevel::Warning, 500));     // beep over, cooling down
    TEST_ASSERT_EQUAL_UINT16(0, b.update(AlertLevel::Warning, 999));
    TEST_ASSERT_EQUAL_UINT16(800, b.update(AlertLevel::Warning, 1000));  // repeats
}

void test_pattern_finishes_even_if_the_alert_clears() {
    AlertBuzzer b(1000);
    b.update(AlertLevel::Danger, 0);
    TEST_ASSERT_EQUAL_UINT16(1200, b.update(AlertLevel::None, 50));
    TEST_ASSERT_EQUAL_UINT16(1200, b.update(AlertLevel::None, 160));   // second beep still plays
    TEST_ASSERT_EQUAL_UINT16(0, b.update(AlertLevel::None, 300));
}

// The original made a new, more severe alert wait out the cooldown.
void test_escalation_does_not_wait_for_the_cooldown() {
    AlertBuzzer b(1000);
    b.update(AlertLevel::Warning, 0);
    TEST_ASSERT_EQUAL_UINT16(2000, b.update(AlertLevel::Critical, 300));
}

void test_same_or_lower_level_cannot_bypass_the_cooldown() {
    AlertBuzzer b(1000);
    b.update(AlertLevel::Critical, 0);
    TEST_ASSERT_EQUAL_UINT16(0, b.update(AlertLevel::Critical, 600));
    TEST_ASSERT_EQUAL_UINT16(0, b.update(AlertLevel::Warning, 700));
    TEST_ASSERT_EQUAL_UINT16(800, b.update(AlertLevel::Warning, 1000));
}

void test_startup_chime_through_alert_buzzer() {
    AlertBuzzer b(1000);
    b.playStartup(0);
    TEST_ASSERT_EQUAL_UINT16(1000, b.update(AlertLevel::None, 10));
    TEST_ASSERT_EQUAL_UINT16(1400, b.update(AlertLevel::None, 130));
    TEST_ASSERT_EQUAL_UINT16(0, b.update(AlertLevel::None, 250));
}

int runTests() {
    UNITY_BEGIN();
    RUN_TEST(test_idle_sequencer_is_silent);
    RUN_TEST(test_warning_is_one_150ms_beep);
    RUN_TEST(test_danger_is_two_beeps_with_a_gap);
    RUN_TEST(test_critical_is_three_beeps);
    RUN_TEST(test_startup_chime);
    RUN_TEST(test_no_pattern_for_no_alert);
    RUN_TEST(test_pattern_survives_clock_wraparound);
    RUN_TEST(test_stop_silences_immediately);
    RUN_TEST(test_no_alert_no_sound);
    RUN_TEST(test_alert_sounds_on_the_first_update);
    RUN_TEST(test_pattern_repeats_only_after_the_cooldown);
    RUN_TEST(test_pattern_finishes_even_if_the_alert_clears);
    RUN_TEST(test_escalation_does_not_wait_for_the_cooldown);
    RUN_TEST(test_same_or_lower_level_cannot_bypass_the_cooldown);
    RUN_TEST(test_startup_chime_through_alert_buzzer);
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
