// config.h - every pin, address, rate, threshold and behaviour switch in one place.
//
// This header is deliberately free of Arduino includes so the hardware-independent
// code in src/core/ (and its unit tests) can use it on a desktop build.
//
// Switches marked [ORIGINAL: x] change behaviour relative to the original sketch
// (archive/f1_helmet_original.ino); the value x restores what the sketch did.
// The reasoning for each default is in docs/review.md.
#pragma once

#include <stdint.h>

namespace cfg {

// ── Pins (ESP32 Dev Module / NodeMCU-32S) ────────────────────────────────────
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;
constexpr uint8_t PIN_GSR     = 34;   // ADC1_CH6, input-only pin
constexpr uint8_t PIN_BUZZER  = 25;   // passive piezo, driven by LEDC PWM

// Optional acknowledge button (active low, internal pull-up). The original
// hardware has none, so this is disabled. GPIO0 (the BOOT button on most dev
// boards) works without extra parts. Alerts can always be acknowledged over
// serial ('a'), see logger.h.
constexpr int8_t PIN_ACK_BUTTON = -1;   // [ORIGINAL: -1]

// ── I2C ──────────────────────────────────────────────────────────────────────
constexpr uint8_t LCD_I2C_ADDR     = 0x3F;   // change to 0x27 if the display stays blank
constexpr uint8_t MMA8451_I2C_ADDR = 0x1D;
// The MAX30102 is fixed at 0x57 by the SparkFun driver.

// 400 kHz is what the original used. The MMA8451 and MAX30102 support it, but a
// PCF8574-based LCD backpack is only specified for 100 kHz. Drop to 100000 if
// the LCD shows corrupted characters (docs/review.md, issue 10).
constexpr uint32_t I2C_CLOCK_HZ = 400000;   // [ORIGINAL: 400000]

// ── Task rates ───────────────────────────────────────────────────────────────
constexpr uint16_t IMU_POLL_HZ     = 200;    // [ORIGINAL: 50] poll at the sensor output rate
constexpr uint16_t ALERT_EVAL_HZ   = 50;     // GSR sampling + alert evaluation
constexpr uint16_t LCD_REFRESH_HZ  = 5;
constexpr uint16_t LOG_HZ          = 10;
constexpr uint16_t PPG_POLL_MS     = 10;     // how often the MAX30102 FIFO is drained
constexpr uint32_t LCD_PAGE_MS     = 3000;   // page rotation period
constexpr uint32_t READY_BANNER_MS = 2000;
constexpr uint32_t SERIAL_BAUD     = 115200;

// One calibration window at start-up (and on the serial 'c' command): the GSR
// baseline and the initial gravity vector are both averaged over it.
constexpr uint32_t CALIBRATION_MS = 5000;

// ── Accelerometer (MMA8451) ──────────────────────────────────────────────────
constexpr float IMU_RANGE_G = 8.0f;

// The Adafruit driver enables the MMA8451 "low noise" bit in begin(). The
// datasheet says that bit limits the measurable signal to +/-4 g whatever range
// is selected. true clears the bit so the +/-8 g range is real.
constexpr bool IMU_CLEAR_LOW_NOISE = true;   // [ORIGINAL: false]

enum class GravityMode : uint8_t {
    FixedSensorZ,   // original: subtract 1 g from the sensor Z axis
    GatedLowPass,   // estimate gravity from the accelerometer itself
};
constexpr GravityMode IMU_GRAVITY_MODE = GravityMode::GatedLowPass;   // [ORIGINAL: FixedSensorZ]

// Gravity estimator tuning (GatedLowPass only). The estimate follows the
// measured vector with time constant TAU, but only while the measured
// magnitude is within GATE of 1 g, so sustained cornering/braking loads are
// not absorbed into "gravity".
constexpr float IMU_GRAVITY_TAU_S  = 1.0f;
constexpr float IMU_GRAVITY_GATE_G = 0.15f;

// Roughly which way the sensor axes point when the helmet is worn: the
// direction of travel expressed in sensor coordinates. It is projected onto
// the horizontal plane found from gravity, so it only needs to be approximate.
// (0,1,0) keeps the original convention: sensor Y = longitudinal, X = lateral.
constexpr float IMU_FORWARD_HINT_X = 0.0f;
constexpr float IMU_FORWARD_HINT_Y = 1.0f;
constexpr float IMU_FORWARD_HINT_Z = 0.0f;

// A raw axis reading within this fraction of full scale is flagged as clipped.
constexpr float IMU_SATURATION_FRACTION = 0.98f;
// This many identical consecutive raw samples means the sensor has stopped
// responding (a live 14-bit accelerometer always shows some noise).
constexpr uint16_t IMU_STUCK_SAMPLES = 100;

// ── Pulse oximeter (MAX30102) ────────────────────────────────────────────────
// 100 Hz conversion rate with 4x on-chip averaging = 25 samples/s out of the
// FIFO. That is exactly the rate the Maxim SpO2 algorithm is written for
// (FreqS = 25, 100 samples = 4 s window); ppg.cpp checks this at compile time.
constexpr uint8_t  PPG_LED_AMPLITUDE  = 0x3F;   // register value, about 12.6 mA (0.2 mA per step)
constexpr uint8_t  PPG_SAMPLE_AVERAGE = 4;
constexpr uint16_t PPG_SAMPLE_RATE_HZ = 100;
constexpr uint16_t PPG_OUTPUT_HZ      = PPG_SAMPLE_RATE_HZ / PPG_SAMPLE_AVERAGE;   // 25
constexpr uint16_t PPG_PULSE_WIDTH_US = 411;
constexpr uint16_t PPG_ADC_RANGE      = 4096;
constexpr uint8_t  PPG_SPO2_WINDOW    = 100;    // samples (4 s)
constexpr uint8_t  PPG_SPO2_STEP      = 25;     // recompute every 25 new samples (1 s)

// Skin-contact detection on the raw IR level. The original used a single
// 50000 threshold on single samples; set ABSENT = PRESENT and DEBOUNCE = 1 to
// get that back. These numbers are untuned for a forehead/temple placement.
constexpr uint32_t PPG_IR_PRESENT       = 50000;
constexpr uint32_t PPG_IR_ABSENT        = 45000;   // [ORIGINAL: 50000]
constexpr uint8_t  PPG_CONTACT_DEBOUNCE = 3;       // [ORIGINAL: 1] consecutive samples

constexpr float PPG_BPM_MIN = 20.0f;   // beat-to-beat rates outside (MIN, MAX) are discarded
constexpr float PPG_BPM_MAX = 220.0f;
// Readings that stop updating are blanked instead of being shown as current.
// 0 disables the timeout (original behaviour: last value shown indefinitely).
constexpr uint32_t PPG_HR_STALE_MS   = 5000;    // [ORIGINAL: 0]
constexpr uint32_t PPG_SPO2_STALE_MS = 10000;   // [ORIGINAL: 0]
constexpr uint32_t PPG_STALL_MS      = 1000;    // no samples at all for this long = sensor fault

// ── GSR (Grove GSR v1.2 on an ESP32 ADC pin) ─────────────────────────────────
constexpr uint8_t GSR_SAMPLES = 8;   // averaged per reading
// true: read calibrated millivolts (eFuse ADC calibration); false: raw 12-bit counts.
constexpr bool  GSR_USE_MILLIVOLTS = true;   // [ORIGINAL: false]
constexpr float GSR_FULL_SCALE     = GSR_USE_MILLIVOLTS ? 3300.0f : 4095.0f;

// Which direction of the signal counts as increased arousal.
// UNVERIFIED: the original treats a RISE as arousal. The resistance formula
// Seeed publishes implies the reading FALLS as skin resistance falls (that is,
// as arousal rises), but their calibration text is ambiguous and this has not
// been checked on the bench. docs/review.md, issue 6c, has the one-minute test
// that settles it. Set false to invert.
constexpr bool GSR_RISE_IS_AROUSAL = true;   // [ORIGINAL: true]

// Slow baseline tracking. 0 = baseline fixed at the calibration value for the
// whole session (original). A value such as 120 lets the baseline follow slow
// drift (sweat build-up, electrode settling) with that time constant.
constexpr float GSR_BASELINE_TAU_S = 0.0f;   // [ORIGINAL: 0]

// Readings at either ADC rail are treated as a wiring fault. Provisional
// values; the ESP32 ADC cannot report much below ~150 mV at 11 dB attenuation.
constexpr float GSR_RAIL_LOW  = GSR_USE_MILLIVOLTS ? 150.0f : 5.0f;
constexpr float GSR_RAIL_HIGH = GSR_USE_MILLIVOLTS ? 3050.0f : 4090.0f;

// ── Alert thresholds (all from the original sketch) ──────────────────────────
constexpr float G_WARN           = 2.5f;
constexpr float G_DANGER         = 4.0f;
constexpr float G_CRITICAL       = 6.0f;
constexpr int   GSR_INDEX_WARN   = 60;   // was STRESS_WARN
constexpr int   GSR_INDEX_DANGER = 80;   // was STRESS_DANGER
constexpr int   HR_LOW           = 40;
constexpr int   HR_HIGH          = 180;

// Hysteresis: an alert level is released only once the value has moved this
// far back past its threshold. 0 everywhere = original behaviour.
constexpr float G_HYSTERESIS         = 0.3f;   // [ORIGINAL: 0]
constexpr int   GSR_INDEX_HYSTERESIS = 5;      // [ORIGINAL: 0]
constexpr int   HR_HYSTERESIS        = 5;      // [ORIGINAL: 0]

// Minimum time an alert level stays shown/logged after its cause has gone.
// The original cleared on the next 20 ms tick, so a short spike was usually
// never drawn on the 5 Hz LCD nor written to the 10 Hz log.
constexpr uint32_t ALERT_HOLD_MS = 3000;   // [ORIGINAL: 0]

// true: CRITICAL stays latched until acknowledged (serial 'a' or the optional
// button). Off by default because the original hardware has no button.
constexpr bool ALERT_LATCH_CRITICAL = false;   // [ORIGINAL: false]

constexpr uint32_t BUZZER_COOLDOWN_MS = 1000;   // minimum gap between pattern starts

}  // namespace cfg
