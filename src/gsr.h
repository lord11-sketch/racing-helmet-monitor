// Grove GSR v1.2 on an ESP32 ADC pin: averaged reading, baseline, arousal index.
#pragma once

#include "telemetry.h"

namespace gsr {

// Configures the ADC pin and takes a first reading. Returns true if that
// reading is off the ADC rails. (A floating pin cannot be told apart from a
// connected sensor in software; see docs/review.md, issue 6a.)
bool begin(Telemetry& t);

// Takes one averaged reading and updates gsrValue, gsrIndex, gsrOk.
// Call at cfg::ALERT_EVAL_HZ.
void sample(Telemetry& t);

// Calibration: every reading between begin and finish is averaged; finish
// stores the mean as the baseline. Returns false if no valid readings were
// collected.
void beginCalibration();
bool finishCalibration(Telemetry& t);

}  // namespace gsr
