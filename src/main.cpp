// Racing Driver Helmet Biometric Monitor
// ESP32 Dev Module (NodeMCU-32S)
//
// Cooperative, non-blocking main loop. Each task runs on its own period and
// returns quickly; nothing after setup() calls delay().
//
//   every loop   buzzer pattern, serial commands, optional button
//   10 ms        drain the MAX30102 FIFO (25 samples/s arrive)
//   5 ms         read the accelerometer (200 Hz, the sensor's output rate)
//   20 ms        read GSR, evaluate alerts
//   100 ms       CSV row
//   200 ms       LCD
//
// Start-up: sensors are probed, then a 5 s calibration window runs inside the
// normal loop (GSR baseline + gravity vector) while everything keeps sampling.

#include <Arduino.h>
#include <Wire.h>

#include "buzzer.h"
#include "config.h"
#include "core/alerts.h"
#include "display.h"
#include "gsr.h"
#include "imu.h"
#include "logger.h"
#include "ppg.h"
#include "telemetry.h"

// The unit-test build provides its own setup()/loop().
#ifndef PIO_UNIT_TESTING

namespace {

Telemetry telemetry;

const core::AlertThresholds kThresholds = {
    cfg::G_WARN,
    cfg::G_DANGER,
    cfg::G_CRITICAL,
    cfg::GSR_INDEX_WARN,
    cfg::GSR_INDEX_DANGER,
    cfg::HR_LOW,
    cfg::HR_HIGH,
    cfg::G_HYSTERESIS,
    cfg::GSR_INDEX_HYSTERESIS,
    cfg::HR_HYSTERESIS,
};
core::AlertEvaluator alertEvaluator(kThresholds);
core::AlertLatch     alertLatch(cfg::ALERT_HOLD_MS, cfg::ALERT_LATCH_CRITICAL);

uint32_t calibrationStartMs = 0;
bool     headerPrinted      = false;

uint32_t lastImuUs   = 0;
uint32_t lastPpgMs   = 0;
uint32_t lastAlertMs = 0;
uint32_t lastLcdMs   = 0;
uint32_t lastLogMs   = 0;

// True (and restarts the period) once `period` has passed since `last`.
bool due(uint32_t& last, uint32_t period, uint32_t now) {
    if (now - last < period) return false;
    last = now;
    return true;
}

void startCalibration(uint32_t nowMs) {
    telemetry.calibrating            = true;
    telemetry.calibrationRemainingMs = cfg::CALIBRATION_MS;
    calibrationStartMs               = nowMs;
    imu::beginCalibration();
    gsr::beginCalibration();
    alertEvaluator.reset();
    alertLatch.acknowledge();
    telemetry.alert = telemetry.alertInstant = core::AlertLevel::None;
    telemetry.alertSources                   = 0;
    logger::info("calibrating for %lu ms: keep the helmet still, electrodes on",
                 static_cast<unsigned long>(cfg::CALIBRATION_MS));
}

void finishCalibration(uint32_t nowMs) {
    telemetry.calibrating            = false;
    telemetry.calibrationRemainingMs = 0;
    imu::finishCalibration();
    const bool gsrCalibrated = gsr::finishCalibration(telemetry);
    imu::resetSessionPeak(telemetry);
    telemetry.gPeakLog = 0.0f;

    logger::info("sensors: MMA8451 %s, MAX30102 %s, GSR %s", telemetry.imuOk ? "OK" : "FAIL",
                 telemetry.ppgOk ? "OK" : "FAIL", gsrCalibrated ? "OK" : "FAIL");
    if (!headerPrinted) {
        logger::printHelp();
        logger::printHeader();
        headerPrinted = true;
    }
    display::showReady(nowMs);
    buzzer::playStartup(nowMs);
}

void evaluateAlerts(uint32_t nowMs) {
    core::AlertInputs in;
    const float       peak = imu::takeAlertPeak();
    in.gDynamic  = telemetry.imuOk ? peak : 0.0f;
    in.gsrIndex  = telemetry.gsrOk ? telemetry.gsrIndex : 0;
    in.heartRate = telemetry.heartRate;

    const core::AlertResult instant = alertEvaluator.update(in);
    const core::AlertResult shown   = alertLatch.update(instant, nowMs);
    telemetry.alertInstant = instant.level;
    telemetry.alert        = shown.level;
    telemetry.alertSources = shown.sources;
}

void handleCommand(logger::Command command, uint32_t nowMs) {
    switch (command) {
        case logger::Command::Acknowledge:
            alertLatch.acknowledge();
            logger::info("alert acknowledged");
            break;
        case logger::Command::ResetPeak:
            imu::resetSessionPeak(telemetry);
            logger::info("session peak reset");
            break;
        case logger::Command::Recalibrate:
            if (!telemetry.calibrating) startCalibration(nowMs);
            break;
        case logger::Command::Help:
            logger::printHelp();
            break;
        case logger::Command::None:
            break;
    }
}

// Optional acknowledge button: active low, acts once per press.
void pollAckButton(uint32_t nowMs) {
    if (cfg::PIN_ACK_BUTTON < 0) return;
    static bool     wasPressed  = false;
    static uint32_t lastEdgeMs  = 0;
    const bool      pressed     = digitalRead(cfg::PIN_ACK_BUTTON) == LOW;
    if (pressed != wasPressed && nowMs - lastEdgeMs > 30) {   // 30 ms debounce
        wasPressed = pressed;
        lastEdgeMs = nowMs;
        if (pressed) handleCommand(logger::Command::Acknowledge, nowMs);
    }
}

}  // namespace

void setup() {
    logger::begin();
    delay(500);   // let the USB-serial bridge settle so the first lines are not lost
    logger::info("Racing Driver Helmet Biometric Monitor");

    Wire.begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL);
    Wire.setClock(cfg::I2C_CLOCK_HZ);

    buzzer::begin();
    if (cfg::PIN_ACK_BUTTON >= 0) pinMode(cfg::PIN_ACK_BUTTON, INPUT_PULLUP);

    // The short delays below only keep boot messages readable; sampling has
    // not started yet, so nothing is lost by them.
    if (!display::begin()) {
        logger::info("WARN no LCD at 0x%02X (try 0x27 in config.h); continuing without a display",
                     cfg::LCD_I2C_ADDR);
    }
    display::message("Helmet Monitor", " Initialising..");
    delay(800);

    if (imu::begin(telemetry)) {
        logger::info("MMA8451 ready");
    } else {
        logger::info("ERR MMA8451 not found at 0x%02X", cfg::MMA8451_I2C_ADDR);
        display::message("Helmet Monitor", " MMA8451 FAIL!");
        delay(1500);
    }

    if (ppg::begin(telemetry)) {
        logger::info("MAX30102 ready");
    } else {
        logger::info("ERR MAX30102 not found");
        display::message("Helmet Monitor", " MAX30102 FAIL!");
        delay(1500);
    }
    // The MAX30102 driver sets the bus speed itself in begin(); set it back so
    // config.h stays the single source of truth.
    Wire.setClock(cfg::I2C_CLOCK_HZ);

    if (gsr::begin(telemetry)) {
        logger::info("GSR input on GPIO%u reads %d %s", cfg::PIN_GSR, telemetry.gsrValue,
                     cfg::GSR_USE_MILLIVOLTS ? "mV" : "counts");
    } else {
        logger::info("WARN GSR input on GPIO%u is at an ADC rail (%d), check wiring", cfg::PIN_GSR,
                     telemetry.gsrValue);
    }

    const uint32_t nowMs = millis();
    lastImuUs            = micros();
    lastPpgMs = lastAlertMs = lastLcdMs = lastLogMs = nowMs;
    startCalibration(nowMs);
}

void loop() {
    const uint32_t nowMs = millis();

    if (due(lastPpgMs, cfg::PPG_POLL_MS, nowMs)) ppg::poll(telemetry, nowMs);

    if (due(lastImuUs, 1000000UL / cfg::IMU_POLL_HZ, micros())) imu::poll(telemetry);

    if (due(lastAlertMs, 1000UL / cfg::ALERT_EVAL_HZ, nowMs)) {
        gsr::sample(telemetry);
        if (telemetry.calibrating) {
            const uint32_t elapsed = nowMs - calibrationStartMs;
            if (elapsed >= cfg::CALIBRATION_MS) {
                finishCalibration(nowMs);
            } else {
                telemetry.calibrationRemainingMs = cfg::CALIBRATION_MS - elapsed;
            }
        } else {
            evaluateAlerts(nowMs);
        }
    }

    buzzer::update(telemetry.calibrating ? core::AlertLevel::None : telemetry.alert, nowMs);

    if (due(lastLcdMs, 1000UL / cfg::LCD_REFRESH_HZ, nowMs)) display::update(telemetry, nowMs);

    // CSV rows start once the first calibration has finished and the header is out.
    if (due(lastLogMs, 1000UL / cfg::LOG_HZ, nowMs) && !telemetry.calibrating) logger::logRow(telemetry, nowMs);

    handleCommand(logger::pollCommand(), nowMs);
    pollAckButton(nowMs);
}

#endif  // PIO_UNIT_TESTING
