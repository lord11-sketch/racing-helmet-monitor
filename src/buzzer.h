// Piezo buzzer on an LEDC PWM channel. Nothing here blocks: update() is called
// every loop and only touches the PWM when the note changes.
#pragma once

#include <stdint.h>

#include "core/alerts.h"

namespace buzzer {

void begin();
void playStartup(uint32_t nowMs);
void update(core::AlertLevel level, uint32_t nowMs);

}  // namespace buzzer
