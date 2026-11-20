#include "buzzer.h"

#include <Arduino.h>

#include "config.h"
#include "core/buzzer_pattern.h"

namespace buzzer {

namespace {

core::AlertBuzzer policy(cfg::BUZZER_COOLDOWN_MS);
uint16_t          currentHz = 0;

const uint8_t kResolutionBits = 8;

// The LEDC API changed in arduino-esp32 3.x (channels are assigned per pin).
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void attach() { ledcAttach(cfg::PIN_BUZZER, 2000, kResolutionBits); }
void writeTone(uint16_t hz) { ledcWriteTone(cfg::PIN_BUZZER, hz); }
#else
const uint8_t kChannel = 0;
void attach() {
    ledcSetup(kChannel, 2000, kResolutionBits);
    ledcAttachPin(cfg::PIN_BUZZER, kChannel);
}
void writeTone(uint16_t hz) { ledcWriteTone(kChannel, hz); }
#endif

}  // namespace

void begin() {
    attach();
    writeTone(0);
}

void playStartup(uint32_t nowMs) { policy.playStartup(nowMs); }

void update(core::AlertLevel level, uint32_t nowMs) {
    const uint16_t hz = policy.update(level, nowMs);
    if (hz != currentHz) {
        writeTone(hz);   // 0 = silence
        currentHz = hz;
    }
}

}  // namespace buzzer
