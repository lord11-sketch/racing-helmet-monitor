// Unit tests for heart-rate averaging and contact detection (src/core/ppg_logic.*).
#include <unity.h>

#include "core/ppg_logic.h"

using core::BeatAverager;
using core::ContactDetector;

void setUp() {}
void tearDown() {}

// ── BeatAverager ─────────────────────────────────────────────────────────────

void test_first_beat_only_sets_the_reference() {
    BeatAverager b(20.0f, 220.0f);
    TEST_ASSERT_FALSE(b.onBeat(1000));
    TEST_ASSERT_EQUAL_INT(0, b.averageBpm());
    TEST_ASSERT_EQUAL_UINT8(0, b.count());
}

// The original averaged over four slots that started at zero, so the first
// 60 bpm interval was reported as 15 bpm.
void test_average_is_not_diluted_by_empty_slots() {
    BeatAverager b(20.0f, 220.0f);
    b.onBeat(1000);
    TEST_ASSERT_TRUE(b.onBeat(2000));
    TEST_ASSERT_EQUAL_INT(60, b.averageBpm());
    TEST_ASSERT_EQUAL_UINT8(1, b.count());
}

void test_average_covers_the_last_four_intervals() {
    BeatAverager b(20.0f, 220.0f);
    b.onBeat(1000);
    b.onBeat(2000);   // 60
    b.onBeat(3000);   // 60
    b.onBeat(3500);   // 120
    TEST_ASSERT_EQUAL_INT(80, b.averageBpm());    // (60 + 60 + 120) / 3
    b.onBeat(4000);   // 120
    TEST_ASSERT_EQUAL_INT(90, b.averageBpm());    // (60 + 60 + 120 + 120) / 4
    b.onBeat(4500);   // 120, replaces the oldest
    TEST_ASSERT_EQUAL_INT(105, b.averageBpm());   // (120 + 60 + 120 + 120) / 4
    b.onBeat(5000);   // 120
    TEST_ASSERT_EQUAL_INT(120, b.averageBpm());
    TEST_ASSERT_EQUAL_UINT8(4, b.count());
}

void test_average_is_rounded_not_truncated() {
    BeatAverager b(20.0f, 220.0f);
    b.onBeat(0);
    b.onBeat(700);   // 85.7 bpm
    TEST_ASSERT_EQUAL_INT(86, b.averageBpm());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 85.71f, b.lastBpm());
}

void test_implausibly_fast_interval_is_rejected() {
    BeatAverager b(20.0f, 220.0f);
    b.onBeat(0);
    TEST_ASSERT_FALSE(b.onBeat(100));   // 600 bpm
    TEST_ASSERT_EQUAL_INT(0, b.averageBpm());
    // The rejected beat still becomes the reference for the next interval.
    TEST_ASSERT_TRUE(b.onBeat(1100));
    TEST_ASSERT_EQUAL_INT(60, b.averageBpm());
}

void test_implausibly_slow_interval_is_rejected() {
    BeatAverager b(20.0f, 220.0f);
    b.onBeat(0);
    TEST_ASSERT_FALSE(b.onBeat(4000));   // 15 bpm
    TEST_ASSERT_FALSE(b.onBeat(7000));   // exactly 20 bpm: the limits are exclusive
    TEST_ASSERT_EQUAL_INT(0, b.averageBpm());
}

void test_reset_forgets_everything() {
    BeatAverager b(20.0f, 220.0f);
    b.onBeat(0);
    b.onBeat(1000);
    b.reset();
    TEST_ASSERT_EQUAL_INT(0, b.averageBpm());
    TEST_ASSERT_FALSE(b.onBeat(50000));   // would be a bogus interval without the reset
    TEST_ASSERT_TRUE(b.onBeat(51000));
    TEST_ASSERT_EQUAL_INT(60, b.averageBpm());
}

void test_drop_reference_keeps_the_average() {
    BeatAverager b(20.0f, 220.0f);
    b.onBeat(0);
    b.onBeat(1000);   // 60
    b.dropReference();
    TEST_ASSERT_FALSE(b.onBeat(10000));
    TEST_ASSERT_EQUAL_INT(60, b.averageBpm());
    TEST_ASSERT_TRUE(b.onBeat(10500));   // 120
    TEST_ASSERT_EQUAL_INT(90, b.averageBpm());
}

void test_interval_across_clock_wraparound() {
    BeatAverager   b(20.0f, 220.0f);
    const uint32_t t = 0xFFFFFE00u;   // 512 ms before the counter wraps
    b.onBeat(t);
    TEST_ASSERT_TRUE(b.onBeat(t + 1000u));
    TEST_ASSERT_EQUAL_INT(60, b.averageBpm());
}

void test_duplicate_timestamp_is_ignored() {
    BeatAverager b(20.0f, 220.0f);
    b.onBeat(5);
    TEST_ASSERT_FALSE(b.onBeat(5));
    TEST_ASSERT_EQUAL_INT(0, b.averageBpm());
}

// ── ContactDetector ──────────────────────────────────────────────────────────

void test_contact_needs_consecutive_samples() {
    ContactDetector c(50000, 45000, 3);
    TEST_ASSERT_FALSE(c.update(60000));
    TEST_ASSERT_FALSE(c.update(60000));
    TEST_ASSERT_TRUE(c.update(60000));
}

void test_contact_debounce_restarts_after_a_gap() {
    ContactDetector c(50000, 45000, 3);
    c.update(60000);
    c.update(60000);
    c.update(10000);   // interrupts the run
    TEST_ASSERT_FALSE(c.update(60000));
    TEST_ASSERT_FALSE(c.update(60000));
    TEST_ASSERT_TRUE(c.update(60000));
}

void test_contact_hysteresis_band() {
    ContactDetector c(50000, 45000, 3);
    c.update(60000);
    c.update(60000);
    c.update(60000);
    // Between the two thresholds: stays in contact however long it lasts.
    for (int i = 0; i < 10; ++i) TEST_ASSERT_TRUE(c.update(47000));
    // Below the lower threshold: released after three samples.
    TEST_ASSERT_TRUE(c.update(44000));
    TEST_ASSERT_TRUE(c.update(44000));
    TEST_ASSERT_FALSE(c.update(44000));
    // And the same in-between level is not enough to regain contact.
    for (int i = 0; i < 10; ++i) TEST_ASSERT_FALSE(c.update(47000));
}

void test_single_threshold_no_debounce_matches_original() {
    ContactDetector c(50000, 50000, 1);
    TEST_ASSERT_TRUE(c.update(50000));
    TEST_ASSERT_FALSE(c.update(49999));
}

void test_zero_debounce_is_treated_as_one() {
    ContactDetector c(50000, 45000, 0);
    TEST_ASSERT_TRUE(c.update(50000));
}

void test_contact_reset() {
    ContactDetector c(50000, 45000, 1);
    c.update(60000);
    c.reset();
    TEST_ASSERT_FALSE(c.present());
}

int runTests() {
    UNITY_BEGIN();
    RUN_TEST(test_first_beat_only_sets_the_reference);
    RUN_TEST(test_average_is_not_diluted_by_empty_slots);
    RUN_TEST(test_average_covers_the_last_four_intervals);
    RUN_TEST(test_average_is_rounded_not_truncated);
    RUN_TEST(test_implausibly_fast_interval_is_rejected);
    RUN_TEST(test_implausibly_slow_interval_is_rejected);
    RUN_TEST(test_reset_forgets_everything);
    RUN_TEST(test_drop_reference_keeps_the_average);
    RUN_TEST(test_interval_across_clock_wraparound);
    RUN_TEST(test_duplicate_timestamp_is_ignored);
    RUN_TEST(test_contact_needs_consecutive_samples);
    RUN_TEST(test_contact_debounce_restarts_after_a_gap);
    RUN_TEST(test_contact_hysteresis_band);
    RUN_TEST(test_single_threshold_no_debounce_matches_original);
    RUN_TEST(test_zero_debounce_is_treated_as_one);
    RUN_TEST(test_contact_reset);
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
