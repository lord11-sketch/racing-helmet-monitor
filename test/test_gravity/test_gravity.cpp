// Unit tests for gravity estimation and axis projection (src/core/gravity.*).
#include <unity.h>

#include "core/gravity.h"

using core::AxisComponents;
using core::AxisFrame;
using core::GravityEstimator;
using core::Vec3;

namespace {

const float kDt = 0.005f;   // 200 Hz

// A sensor at rest, pitched 30 degrees: 1 g split between Y and Z.
const Vec3 kTilted30 = {0.0f, 0.5f, 0.8660254f};
const Vec3 kLevel    = {0.0f, 0.0f, 1.0f};

}  // namespace

void setUp() {}
void tearDown() {}

// ── Fixed mode (the original sketch's behaviour) ─────────────────────────────

void test_fixed_mode_removes_gravity_when_level() {
    GravityEstimator e(false, 1.0f, 0.15f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, core::norm(e.update(kLevel, kDt)));

    const Vec3 braking = {0.0f, 1.0f, 1.0f};
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, core::norm(e.update(braking, kDt)));
}

// Documents the bug this module fixes: with a fixed "Z is down" assumption a
// stationary helmet tilted 30 degrees reports 2*sin(15 deg) = 0.52 g.
void test_fixed_mode_reports_phantom_g_when_tilted() {
    GravityEstimator e(false, 1.0f, 0.15f);
    for (int i = 0; i < 2000; ++i) e.update(kTilted30, kDt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5176f, core::norm(e.update(kTilted30, kDt)));
}

// ── Adaptive mode ────────────────────────────────────────────────────────────

void test_adaptive_mode_converges_on_a_tilted_rest_position() {
    GravityEstimator e(true, 1.0f, 0.15f);
    Vec3 dynamic = e.update(kTilted30, kDt);
    for (int i = 0; i < 2000; ++i) dynamic = e.update(kTilted30, kDt);   // 10 s = 10 time constants

    TEST_ASSERT_TRUE(e.gateOpen());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, core::norm(dynamic));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, e.gravity().y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.866f, e.gravity().z);
}

void test_single_step_follows_first_order_low_pass() {
    // tau = dt gives alpha = 0.5: the estimate moves half-way to the sample.
    GravityEstimator e(true, 1.0f, 0.15f);
    const Vec3 sample = {0.0f, 0.1f, 1.0f};   // magnitude 1.005, inside the gate
    e.update(sample, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.05f, e.gravity().y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, e.gravity().z);
}

void test_sustained_load_is_not_absorbed_into_gravity() {
    GravityEstimator e(true, 1.0f, 0.15f);
    const Vec3 cornering = {3.0f, 0.0f, 1.0f};   // 3 g lateral, magnitude 3.16
    Vec3 dynamic = e.update(cornering, kDt);
    for (int i = 0; i < 2000; ++i) dynamic = e.update(cornering, kDt);   // 10 s of it

    TEST_ASSERT_FALSE(e.gateOpen());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, core::norm(dynamic));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, e.gravity().x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, e.gravity().z);

    // Back on the straight the reading is clean straight away.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, core::norm(e.update(kLevel, kDt)));
}

void test_reset_sets_the_estimate() {
    GravityEstimator e(true, 1.0f, 0.15f);
    e.reset(kTilted30);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, core::norm(e.update(kTilted30, kDt)));
}

void test_zero_dt_changes_nothing() {
    GravityEstimator e(true, 1.0f, 0.15f);
    e.update(kTilted30, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, e.gravity().y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, e.gravity().z);
}

// ── Axis frame ───────────────────────────────────────────────────────────────

void test_level_frame_matches_sensor_axes() {
    const Vec3      hint = {0.0f, 1.0f, 0.0f};
    const AxisFrame f    = core::makeAxisFrame(kLevel, hint);
    TEST_ASSERT_TRUE(f.valid);

    const Vec3           v = {0.5f, 0.2f, 0.1f};
    const AxisComponents c = core::projectOnFrame(v, f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, c.lateral);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, c.longitudinal);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, c.vertical);
}

void test_tilted_frame_keeps_forward_horizontal() {
    // Sensor pitched 30 degrees. Horizontal-forward in sensor coordinates is
    // then (0, cos30, -sin30).
    const Vec3      hint = {0.0f, 1.0f, 0.0f};
    const AxisFrame f    = core::makeAxisFrame(kTilted30, hint);
    TEST_ASSERT_TRUE(f.valid);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.8660254f, f.forward.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.5f, f.forward.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, f.lateral.x);

    // 2 g of pure braking along that direction.
    const Vec3           braking = {0.0f, 1.7320508f, -1.0f};
    const AxisComponents c       = core::projectOnFrame(braking, f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, c.longitudinal);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.lateral);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.vertical);
}

void test_frame_ignores_gravity_magnitude() {
    const Vec3      hint   = {0.0f, 1.0f, 0.0f};
    const Vec3      twoG   = {0.0f, 0.0f, 2.0f};
    const AxisFrame f      = core::makeAxisFrame(twoG, hint);
    TEST_ASSERT_TRUE(f.valid);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, f.up.z);
}

void test_vertical_hint_falls_back_to_sensor_axes() {
    const Vec3      hint     = {0.0f, 1.0f, 0.0f};
    const Vec3      onItsEnd = {0.0f, 1.0f, 0.0f};   // gravity along the hint
    const AxisFrame f        = core::makeAxisFrame(onItsEnd, hint);
    TEST_ASSERT_FALSE(f.valid);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, f.lateral.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, f.forward.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, f.up.z);
}

void test_zero_gravity_falls_back_to_sensor_axes() {
    const Vec3 hint = {0.0f, 1.0f, 0.0f};
    const Vec3 zero = {0.0f, 0.0f, 0.0f};
    TEST_ASSERT_FALSE(core::makeAxisFrame(zero, hint).valid);
}

int runTests() {
    UNITY_BEGIN();
    RUN_TEST(test_fixed_mode_removes_gravity_when_level);
    RUN_TEST(test_fixed_mode_reports_phantom_g_when_tilted);
    RUN_TEST(test_adaptive_mode_converges_on_a_tilted_rest_position);
    RUN_TEST(test_single_step_follows_first_order_low_pass);
    RUN_TEST(test_sustained_load_is_not_absorbed_into_gravity);
    RUN_TEST(test_reset_sets_the_estimate);
    RUN_TEST(test_zero_dt_changes_nothing);
    RUN_TEST(test_level_frame_matches_sensor_axes);
    RUN_TEST(test_tilted_frame_keeps_forward_horizontal);
    RUN_TEST(test_frame_ignores_gravity_magnitude);
    RUN_TEST(test_vertical_hint_falls_back_to_sensor_axes);
    RUN_TEST(test_zero_gravity_falls_back_to_sensor_axes);
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
