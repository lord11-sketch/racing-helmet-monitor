// Unit tests for the GSR arousal index, baseline tracking and the calibration
// mean (src/core/gsr_index.*, src/core/mean_accumulator.h).
#include <math.h>
#include <unity.h>

#include "core/gsr_index.h"
#include "core/mean_accumulator.h"

using core::BaselineTracker;
using core::gsrIndex;
using core::gsrInRange;
using core::MeanAccumulator;

void setUp() {}
void tearDown() {}

// ── Index, "rise means arousal" (the original sketch's polarity) ─────────────

void test_reading_at_baseline_is_zero() {
    TEST_ASSERT_EQUAL_INT(0, gsrIndex(2000.0f, 2000.0f, 4095.0f, true));
}

void test_rising_index_scales_over_the_headroom_above_baseline() {
    // Baseline 2000 of 4095 leaves 2095 counts of headroom.
    TEST_ASSERT_EQUAL_INT(50, gsrIndex(3047.5f, 2000.0f, 4095.0f, true));    // half of it
    TEST_ASSERT_EQUAL_INT(100, gsrIndex(4095.0f, 2000.0f, 4095.0f, true));   // all of it
    TEST_ASSERT_EQUAL_INT(25, gsrIndex(2538.4f, 2000.0f, 4095.0f, true));    // 25.7 %, truncated
}

void test_rising_index_is_clamped() {
    TEST_ASSERT_EQUAL_INT(0, gsrIndex(1000.0f, 2000.0f, 4095.0f, true));     // below baseline
    TEST_ASSERT_EQUAL_INT(100, gsrIndex(5000.0f, 2000.0f, 4095.0f, true));   // beyond full scale
}

void test_index_works_in_millivolts() {
    TEST_ASSERT_EQUAL_INT(50, gsrIndex(2475.0f, 1650.0f, 3300.0f, true));
}

// ── Index, "fall means arousal" ──────────────────────────────────────────────

void test_falling_index_scales_over_the_headroom_below_baseline() {
    TEST_ASSERT_EQUAL_INT(50, gsrIndex(1000.0f, 2000.0f, 4095.0f, false));
    TEST_ASSERT_EQUAL_INT(100, gsrIndex(0.0f, 2000.0f, 4095.0f, false));
    TEST_ASSERT_EQUAL_INT(0, gsrIndex(2500.0f, 2000.0f, 4095.0f, false));
}

// ── Degenerate inputs ────────────────────────────────────────────────────────

// The original divided by (4095 - baseline) with no guard.
void test_no_headroom_gives_zero_instead_of_dividing_by_zero() {
    TEST_ASSERT_EQUAL_INT(0, gsrIndex(4095.0f, 4095.0f, 4095.0f, true));
    TEST_ASSERT_EQUAL_INT(0, gsrIndex(4095.0f, 4080.0f, 4095.0f, true));   // under 1 % headroom
    TEST_ASSERT_EQUAL_INT(0, gsrIndex(0.0f, 0.0f, 4095.0f, false));
}

void test_nan_reading_gives_zero() {
    TEST_ASSERT_EQUAL_INT(0, gsrIndex(NAN, 2000.0f, 4095.0f, true));
}

// ── Rail check ───────────────────────────────────────────────────────────────

void test_in_range_excludes_the_rails() {
    TEST_ASSERT_TRUE(gsrInRange(1650.0f, 150.0f, 3050.0f));
    TEST_ASSERT_FALSE(gsrInRange(150.0f, 150.0f, 3050.0f));
    TEST_ASSERT_FALSE(gsrInRange(3050.0f, 150.0f, 3050.0f));
    TEST_ASSERT_FALSE(gsrInRange(100.0f, 150.0f, 3050.0f));
}

// ── Baseline tracker ─────────────────────────────────────────────────────────

void test_baseline_is_fixed_when_tracking_is_off() {
    BaselineTracker b(0.0f);
    b.set(1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1000.0f, b.update(2000.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1000.0f, b.value());
}

void test_baseline_follows_as_a_first_order_low_pass() {
    BaselineTracker b(10.0f);
    b.set(1000.0f);
    // dt = tau gives alpha = 0.5: half-way to the reading.
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1500.0f, b.update(2000.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 1750.0f, b.update(2000.0f, 10.0f));
}

void test_baseline_ignores_zero_dt() {
    BaselineTracker b(10.0f);
    b.set(1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1000.0f, b.update(2000.0f, 0.0f));
}

// ── Calibration mean ─────────────────────────────────────────────────────────

void test_mean_of_no_samples_is_the_fallback() {
    MeanAccumulator m;
    TEST_ASSERT_EQUAL_UINT32(0, m.count());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 7.0f, m.mean(7.0f));
}

void test_mean_and_reset() {
    MeanAccumulator m;
    m.add(1.0f);
    m.add(2.0f);
    m.add(3.0f);
    TEST_ASSERT_EQUAL_UINT32(3, m.count());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, m.mean());
    m.reset();
    TEST_ASSERT_EQUAL_UINT32(0, m.count());
}

int runTests() {
    UNITY_BEGIN();
    RUN_TEST(test_reading_at_baseline_is_zero);
    RUN_TEST(test_rising_index_scales_over_the_headroom_above_baseline);
    RUN_TEST(test_rising_index_is_clamped);
    RUN_TEST(test_index_works_in_millivolts);
    RUN_TEST(test_falling_index_scales_over_the_headroom_below_baseline);
    RUN_TEST(test_no_headroom_gives_zero_instead_of_dividing_by_zero);
    RUN_TEST(test_nan_reading_gives_zero);
    RUN_TEST(test_in_range_excludes_the_rails);
    RUN_TEST(test_baseline_is_fixed_when_tracking_is_off);
    RUN_TEST(test_baseline_follows_as_a_first_order_low_pass);
    RUN_TEST(test_baseline_ignores_zero_dt);
    RUN_TEST(test_mean_of_no_samples_is_the_fallback);
    RUN_TEST(test_mean_and_reset);
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
