#include "imu.h"

#include <Adafruit_MMA8451.h>
#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "core/gravity.h"
#include "core/mean_accumulator.h"
#include "logger.h"

namespace imu {

namespace {

Adafruit_MMA8451 mma;

const bool kAdaptive = cfg::IMU_GRAVITY_MODE == cfg::GravityMode::GatedLowPass;
core::GravityEstimator estimator(kAdaptive, cfg::IMU_GRAVITY_TAU_S, cfg::IMU_GRAVITY_GATE_G);
const core::Vec3 kForwardHint = {cfg::IMU_FORWARD_HINT_X, cfg::IMU_FORWARD_HINT_Y, cfg::IMU_FORWARD_HINT_Z};

bool     present    = false;
uint32_t lastPollUs = 0;
float    alertPeak  = 0.0f;

// Stuck-sensor detection
int16_t  lastRaw[3]  = {0, 0, 0};
uint16_t stuckCount  = 0;

// Calibration
bool                  calibrating = false;
core::MeanAccumulator meanX, meanY, meanZ;

// Raw 14-bit counts per g at +/-8 g is 1024. With the low-noise bit left set
// the device clips at +/-4 g instead.
int16_t saturationCounts() {
    const float clipG = cfg::IMU_CLEAR_LOW_NOISE ? cfg::IMU_RANGE_G : 4.0f;
    return static_cast<int16_t>(clipG * 1024.0f * cfg::IMU_SATURATION_FRACTION);
}

}  // namespace

bool begin(Telemetry& t) {
    present = mma.begin(cfg::MMA8451_I2C_ADDR);
    if (present) {
        mma.setRange(MMA8451_RANGE_8_G);
        mma.setDataRate(MMA8451_DATARATE_200_HZ);
        if (cfg::IMU_CLEAR_LOW_NOISE) {
            // CTRL_REG1 may only be changed in standby. Rewrite it with the same
            // data rate, ACTIVE set and LNOISE (bit 2) clear.
            mma.writeRegister8(MMA8451_REG_CTRL_REG1, 0x00);
            mma.writeRegister8(MMA8451_REG_CTRL_REG1, (MMA8451_DATARATE_200_HZ << 3) | 0x01);
        }
        lastPollUs = micros();
    }
    t.imuOk = present;
    return present;
}

void poll(Telemetry& t) {
    if (!present) return;

    mma.read();
    const uint32_t nowUs = micros();
    float dtS = static_cast<float>(nowUs - lastPollUs) * 1e-6f;
    lastPollUs = nowUs;
    if (dtS > 0.1f) dtS = 0.1f;   // after a long stall, do not let one sample swing the filter

    // A failed I2C read leaves the driver returning the same numbers forever.
    if (mma.x == lastRaw[0] && mma.y == lastRaw[1] && mma.z == lastRaw[2]) {
        if (stuckCount < cfg::IMU_STUCK_SAMPLES) ++stuckCount;
    } else {
        stuckCount = 0;
    }
    lastRaw[0] = mma.x;
    lastRaw[1] = mma.y;
    lastRaw[2] = mma.z;
    t.imuOk = stuckCount < cfg::IMU_STUCK_SAMPLES;

    const int16_t sat = saturationCounts();
    if (abs(mma.x) >= sat || abs(mma.y) >= sat || abs(mma.z) >= sat) t.imuSaturated = true;

    const core::Vec3 accel = {mma.x_g, mma.y_g, mma.z_g};
    if (calibrating) {
        meanX.add(accel.x);
        meanY.add(accel.y);
        meanZ.add(accel.z);
    }

    const core::Vec3           dynamic = estimator.update(accel, dtS);
    const core::AxisFrame      frame   = core::makeAxisFrame(estimator.gravity(), kForwardHint);
    const core::AxisComponents parts   = core::projectOnFrame(dynamic, frame);

    t.gTotal    = core::norm(dynamic);
    t.gLateral  = fabsf(parts.lateral);
    t.gLong     = fabsf(parts.longitudinal);
    t.gVertical = parts.vertical;

    // Peaks are not tracked while calibrating: the gravity vector is not known yet.
    if (!calibrating) {
        if (t.gTotal > t.gPeakSession) t.gPeakSession = t.gTotal;
        if (t.gTotal > t.gPeakLog) t.gPeakLog = t.gTotal;
        if (t.gTotal > alertPeak) alertPeak = t.gTotal;
    }
}

float takeAlertPeak() {
    const float peak = alertPeak;
    alertPeak = 0.0f;
    return peak;
}

void resetSessionPeak(Telemetry& t) { t.gPeakSession = 0.0f; }

void beginCalibration() {
    meanX.reset();
    meanY.reset();
    meanZ.reset();
    calibrating = true;
}

bool finishCalibration() {
    calibrating = false;
    alertPeak   = 0.0f;
    if (!present || meanX.count() == 0) return false;

    const core::Vec3 mean = {meanX.mean(), meanY.mean(), meanZ.mean()};
    const float      mag  = core::norm(mean);
    logger::info("IMU rest vector: x=%.3f y=%.3f z=%.3f |a|=%.3f g (%lu samples)",
                 static_cast<double>(mean.x), static_cast<double>(mean.y), static_cast<double>(mean.z),
                 static_cast<double>(mag), static_cast<unsigned long>(meanX.count()));

    // If the mean is not close to 1 g the helmet was moving (or the sensor is
    // faulty) and the vector cannot be trusted as "down".
    const bool plausible = mag > 0.8f && mag < 1.2f;
    if (!plausible) {
        logger::info("WARN IMU calibration rejected: mean is not ~1 g, was the helmet moving?");
        return false;
    }
    if (kAdaptive) estimator.reset(mean);
    return true;
}

}  // namespace imu
