// Unit tests for the alert decision logic (src/core/alerts.*).
#include <math.h>
#include <unity.h>

#include "core/alerts.h"

using core::AlertEvaluator;
using core::AlertInputs;
using core::AlertLatch;
using core::AlertLevel;
using core::AlertResult;
using core::AlertThresholds;

namespace {

// The thresholds from the original sketch, with a chosen amount of hysteresis.
AlertThresholds thresholds(float gHyst, int gsrHyst, int hrHyst) {
    AlertThresholds t;
    t.gWarn         = 2.5f;
    t.gDanger       = 4.0f;
    t.gCritical     = 6.0f;
    t.gsrWarn       = 60;
    t.gsrDanger     = 80;
    t.hrLow         = 40;
    t.hrHigh        = 180;
    t.gHysteresis   = gHyst;
    t.gsrHysteresis = gsrHyst;
    t.hrHysteresis  = hrHyst;
    return t;
}

AlertInputs inputs(float g, int gsr, int hr) {
    AlertInputs in;
    in.gDynamic  = g;
    in.gsrIndex  = gsr;
    in.heartRate = hr;
    return in;
}

// One evaluation with no history and no hysteresis: the original sketch's rule.
AlertResult instant(float g, int gsr, int hr) {
    AlertEvaluator evaluator(thresholds(0.0f, 0, 0));
    return evaluator.update(inputs(g, gsr, hr));
}

AlertResult result(AlertLevel level, uint8_t sources = 0) {
    AlertResult r;
    r.level   = level;
    r.sources = sources;
    return r;
}

void assertLevel(AlertLevel expected, AlertLevel actual) {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(expected), static_cast<int>(actual));
}

}  // namespace

void setUp() {}
void tearDown() {}

// ── Instantaneous rule ───────────────────────────────────────────────────────

void test_quiet_inputs_give_no_alert() {
    const AlertResult r = instant(0.3f, 10, 72);
    assertLevel(AlertLevel::None, r.level);
    TEST_ASSERT_EQUAL_UINT8(0, r.sources);
}

void test_g_thresholds() {
    assertLevel(AlertLevel::None, instant(2.4f, 0, 0).level);
    assertLevel(AlertLevel::Warning, instant(2.5f, 0, 0).level);
    assertLevel(AlertLevel::Warning, instant(3.9f, 0, 0).level);
    assertLevel(AlertLevel::Danger, instant(4.0f, 0, 0).level);
    assertLevel(AlertLevel::Danger, instant(5.9f, 0, 0).level);
    assertLevel(AlertLevel::Critical, instant(6.0f, 0, 0).level);
    TEST_ASSERT_EQUAL_UINT8(core::kSourceG, instant(6.0f, 0, 0).sources);
}

void test_gsr_thresholds_never_reach_critical() {
    assertLevel(AlertLevel::None, instant(0.0f, 59, 0).level);
    assertLevel(AlertLevel::Warning, instant(0.0f, 60, 0).level);
    assertLevel(AlertLevel::Danger, instant(0.0f, 80, 0).level);
    assertLevel(AlertLevel::Danger, instant(0.0f, 100, 0).level);
    TEST_ASSERT_EQUAL_UINT8(core::kSourceGsr, instant(0.0f, 100, 0).sources);
}

void test_heart_rate_limits_are_exclusive() {
    assertLevel(AlertLevel::None, instant(0.0f, 0, 40).level);
    assertLevel(AlertLevel::None, instant(0.0f, 0, 180).level);
    assertLevel(AlertLevel::Danger, instant(0.0f, 0, 39).level);
    assertLevel(AlertLevel::Danger, instant(0.0f, 0, 181).level);
    TEST_ASSERT_EQUAL_UINT8(core::kSourceHr, instant(0.0f, 0, 181).sources);
}

void test_no_heart_rate_reading_never_alerts() {
    assertLevel(AlertLevel::None, instant(0.0f, 0, 0).level);
}

void test_highest_source_wins_and_all_sources_are_reported() {
    const AlertResult r = instant(6.5f, 70, 200);
    assertLevel(AlertLevel::Critical, r.level);
    TEST_ASSERT_EQUAL_UINT8(core::kSourceG | core::kSourceGsr | core::kSourceHr, r.sources);

    // Heart rate lifts a g-force Warning to Danger.
    assertLevel(AlertLevel::Danger, instant(3.0f, 0, 200).level);
}

void test_nan_g_does_not_alert() {
    assertLevel(AlertLevel::None, instant(NAN, 0, 0).level);
}

// ── Hysteresis ───────────────────────────────────────────────────────────────

void test_g_hysteresis_holds_level_inside_band() {
    AlertEvaluator e(thresholds(0.3f, 0, 0));
    assertLevel(AlertLevel::Danger, e.update(inputs(4.1f, 0, 0)).level);
    assertLevel(AlertLevel::Danger, e.update(inputs(3.8f, 0, 0)).level);    // above 4.0 - 0.3
    assertLevel(AlertLevel::Warning, e.update(inputs(3.5f, 0, 0)).level);   // below the band
    assertLevel(AlertLevel::Warning, e.update(inputs(2.3f, 0, 0)).level);   // above 2.5 - 0.3
    assertLevel(AlertLevel::None, e.update(inputs(2.0f, 0, 0)).level);
}

void test_g_can_fall_several_levels_at_once() {
    AlertEvaluator e(thresholds(0.3f, 0, 0));
    assertLevel(AlertLevel::Critical, e.update(inputs(7.0f, 0, 0)).level);
    assertLevel(AlertLevel::None, e.update(inputs(0.2f, 0, 0)).level);
}

void test_without_hysteresis_level_follows_value() {
    AlertEvaluator e(thresholds(0.0f, 0, 0));
    assertLevel(AlertLevel::Danger, e.update(inputs(4.1f, 0, 0)).level);
    assertLevel(AlertLevel::Warning, e.update(inputs(3.9f, 0, 0)).level);
}

void test_gsr_hysteresis() {
    AlertEvaluator e(thresholds(0.0f, 5, 0));
    assertLevel(AlertLevel::Warning, e.update(inputs(0.0f, 61, 0)).level);
    assertLevel(AlertLevel::Warning, e.update(inputs(0.0f, 57, 0)).level);   // above 60 - 5
    assertLevel(AlertLevel::None, e.update(inputs(0.0f, 54, 0)).level);
}

void test_heart_rate_hysteresis_high_and_low() {
    AlertEvaluator e(thresholds(0.0f, 0, 5));
    assertLevel(AlertLevel::Danger, e.update(inputs(0.0f, 0, 181)).level);
    assertLevel(AlertLevel::Danger, e.update(inputs(0.0f, 0, 177)).level);   // above 180 - 5
    assertLevel(AlertLevel::None, e.update(inputs(0.0f, 0, 174)).level);

    assertLevel(AlertLevel::Danger, e.update(inputs(0.0f, 0, 39)).level);
    assertLevel(AlertLevel::Danger, e.update(inputs(0.0f, 0, 43)).level);    // below 40 + 5
    assertLevel(AlertLevel::None, e.update(inputs(0.0f, 0, 46)).level);
}

void test_losing_the_heart_rate_reading_clears_its_alert() {
    AlertEvaluator e(thresholds(0.0f, 0, 5));
    assertLevel(AlertLevel::Danger, e.update(inputs(0.0f, 0, 190)).level);
    assertLevel(AlertLevel::None, e.update(inputs(0.0f, 0, 0)).level);
}

// ── Hold / latch ─────────────────────────────────────────────────────────────

void test_short_spike_is_held_for_the_hold_time() {
    AlertLatch latch(3000, false);
    assertLevel(AlertLevel::Critical, latch.update(result(AlertLevel::Critical, core::kSourceG), 1000).level);
    // The spike is over 20 ms later, but the alert stays up...
    AlertResult r = latch.update(result(AlertLevel::None), 1020);
    assertLevel(AlertLevel::Critical, r.level);
    TEST_ASSERT_EQUAL_UINT8(core::kSourceG, r.sources);
    assertLevel(AlertLevel::Critical, latch.update(result(AlertLevel::None), 3999).level);
    // ...until 3000 ms after the last time it was true.
    assertLevel(AlertLevel::None, latch.update(result(AlertLevel::None), 4000).level);
}

void test_hold_restarts_while_cause_persists() {
    AlertLatch latch(3000, false);
    latch.update(result(AlertLevel::Danger), 0);
    latch.update(result(AlertLevel::Danger), 1000);
    assertLevel(AlertLevel::Danger, latch.update(result(AlertLevel::None), 3500).level);
    assertLevel(AlertLevel::None, latch.update(result(AlertLevel::None), 4000).level);
}

void test_hold_releases_to_the_current_lower_level() {
    AlertLatch latch(3000, false);
    latch.update(result(AlertLevel::Critical, core::kSourceG), 0);
    assertLevel(AlertLevel::Critical, latch.update(result(AlertLevel::Warning, core::kSourceGsr), 2000).level);
    const AlertResult r = latch.update(result(AlertLevel::Warning, core::kSourceGsr), 3000);
    assertLevel(AlertLevel::Warning, r.level);
    TEST_ASSERT_EQUAL_UINT8(core::kSourceGsr, r.sources);
}

void test_escalation_passes_straight_through() {
    AlertLatch latch(3000, false);
    latch.update(result(AlertLevel::Warning), 0);
    assertLevel(AlertLevel::Critical, latch.update(result(AlertLevel::Critical), 10).level);
}

void test_zero_hold_is_a_pass_through() {
    AlertLatch latch(0, false);
    assertLevel(AlertLevel::Critical, latch.update(result(AlertLevel::Critical), 0).level);
    assertLevel(AlertLevel::None, latch.update(result(AlertLevel::None), 1).level);
}

void test_hold_survives_millis_wraparound() {
    AlertLatch     latch(3000, false);
    const uint32_t start = 0xFFFFFF00u;   // 256 ms before the counter wraps
    latch.update(result(AlertLevel::Danger), start);
    assertLevel(AlertLevel::Danger, latch.update(result(AlertLevel::None), start + 2999u).level);
    assertLevel(AlertLevel::None, latch.update(result(AlertLevel::None), start + 3000u).level);
}

void test_latched_critical_needs_acknowledge() {
    AlertLatch latch(3000, true);
    latch.update(result(AlertLevel::Critical), 0);
    assertLevel(AlertLevel::Critical, latch.update(result(AlertLevel::None), 1000000).level);
    latch.acknowledge();
    assertLevel(AlertLevel::None, latch.update(result(AlertLevel::None), 1000020).level);
}

void test_latch_only_applies_to_critical() {
    AlertLatch latch(3000, true);
    latch.update(result(AlertLevel::Danger), 0);
    assertLevel(AlertLevel::None, latch.update(result(AlertLevel::None), 3000).level);
}

void test_acknowledge_cannot_silence_a_live_alert() {
    AlertLatch latch(3000, true);
    latch.update(result(AlertLevel::Critical), 0);
    latch.acknowledge();
    assertLevel(AlertLevel::Critical, latch.update(result(AlertLevel::Critical), 20).level);
}

int runTests() {
    UNITY_BEGIN();
    RUN_TEST(test_quiet_inputs_give_no_alert);
    RUN_TEST(test_g_thresholds);
    RUN_TEST(test_gsr_thresholds_never_reach_critical);
    RUN_TEST(test_heart_rate_limits_are_exclusive);
    RUN_TEST(test_no_heart_rate_reading_never_alerts);
    RUN_TEST(test_highest_source_wins_and_all_sources_are_reported);
    RUN_TEST(test_nan_g_does_not_alert);
    RUN_TEST(test_g_hysteresis_holds_level_inside_band);
    RUN_TEST(test_g_can_fall_several_levels_at_once);
    RUN_TEST(test_without_hysteresis_level_follows_value);
    RUN_TEST(test_gsr_hysteresis);
    RUN_TEST(test_heart_rate_hysteresis_high_and_low);
    RUN_TEST(test_losing_the_heart_rate_reading_clears_its_alert);
    RUN_TEST(test_short_spike_is_held_for_the_hold_time);
    RUN_TEST(test_hold_restarts_while_cause_persists);
    RUN_TEST(test_hold_releases_to_the_current_lower_level);
    RUN_TEST(test_escalation_passes_straight_through);
    RUN_TEST(test_zero_hold_is_a_pass_through);
    RUN_TEST(test_hold_survives_millis_wraparound);
    RUN_TEST(test_latched_critical_needs_acknowledge);
    RUN_TEST(test_latch_only_applies_to_critical);
    RUN_TEST(test_acknowledge_cannot_silence_a_live_alert);
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
