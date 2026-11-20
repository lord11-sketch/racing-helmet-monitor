// MMA8451 accelerometer: sampling, gravity removal, peak tracking.
#pragma once

#include "telemetry.h"

namespace imu {

// Finds and configures the sensor (+/-8 g, 200 Hz). Returns false if absent.
bool begin(Telemetry& t);

// Reads one sample and updates the g-force fields of `t`. Call at
// cfg::IMU_POLL_HZ. Does nothing if the sensor was not found.
void poll(Telemetry& t);

// Largest gTotal seen since the previous call (0 if none). The alert logic
// uses this so a peak between two alert ticks is not lost.
float takeAlertPeak();

void resetSessionPeak(Telemetry& t);

// Calibration: between begin and finish every sample is averaged; finish uses
// the mean as the gravity vector (GatedLowPass mode only) and reports whether
// the helmet was still enough for the result to be trusted.
void beginCalibration();
bool finishCalibration();

}  // namespace imu
