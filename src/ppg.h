// MAX30102 pulse oximeter: heart rate from beat-to-beat timing, SpO2 from the
// Maxim reference algorithm on a rolling 4 s window.
#pragma once

#include "telemetry.h"

namespace ppg {

// Finds and configures the sensor. Returns false if absent.
bool begin(Telemetry& t);

// Drains every sample waiting in the sensor FIFO and updates heartRate, spo2,
// hrMaxim, ppgContact and ppgOk. Never waits for data. Call every
// cfg::PPG_POLL_MS.
void poll(Telemetry& t, uint32_t nowMs);

}  // namespace ppg
