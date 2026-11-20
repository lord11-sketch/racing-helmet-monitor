# Code review: from single sketch to this repository

This document records every issue found in the original sketch ([`archive/f1_helmet_original.ino`](../archive/f1_helmet_original.ino)), the evidence for it, what was changed, and how to check the change on real hardware.

Line numbers refer to the archived file.

## What has and has not been verified

| Check | Result |
|---|---|
| Firmware builds (`pio run`, ESP32 Dev Module, pinned platform and libraries) | Yes, no warnings from project sources |
| Unit tests compile for the ESP32 target and the desktop code path passes a syntax check | Yes |
| Unit tests executed | **No.** The review machine had no host C++ compiler. Run `pio test -e native` |
| Firmware run on the hardware | **No.** Every "how to verify" step below is still to do |
| Findings about library and sensor behaviour | From reading the pinned library sources and the MMA8451Q datasheet, not from measurement |

Nothing in this document is a measured result. Timings such as "about 80 ms" are derived from the source code and the sensor's configured sample rate.

## Summary

Severity: **High** = wrong numbers or missed events in normal use; **Medium** = wrong in some conditions, or undefined behaviour; **Low** = robustness, clarity, maintenance.

| # | Issue | Severity | Status |
|---|---|---|---|
| 1 | Gravity compensation assumes the sensor Z axis is vertical | High | Fixed by default |
| 2 | Blocking calls stall sampling (buzzer `delay()`, start-up waits) | High | Fixed |
| 2a | `getIR()`/`getRed()` each wait for a new sample: loop runs at roughly 12 Hz, red and IR come from different samples | High | Fixed |
| 3 | Stray `%` in a `printf` format (undefined behaviour) | Medium | Fixed |
| 4 | PPG sample-rate and LED-current comments are wrong (the configuration itself is right) | Low | Comments corrected, checked at compile time |
| 5 | Heart-rate averaging: diluted start, bogus first interval, late no-contact check, two heart-rate sources, stale values | Medium | Fixed |
| 6a | GSR "sensor OK" check can never fail | Medium | Partly fixed; needs a hardware pull-down to finish |
| 6b | Raw ESP32 ADC counts are non-linear | Low | Calibrated millivolts by default |
| 6c | Polarity of the GSR signal | **Unverified** | Switch provided, original polarity kept |
| 6d | GSR baseline fixed for the whole session | Low | Optional tracking, off by default |
| 6e | "Stress index" overstates what is measured; division by zero at full-scale baseline | Medium | Renamed; guarded |
| 7a | Accelerometer is effectively limited to ±4 g, not ±8 g, by the driver's low-noise setting | High | Fixed by default |
| 7b | Peaks missed: 200 Hz sensor polled at 50 Hz (in practice slower, see 2a) | High | Polled at 200 Hz; window peak used for alerts |
| 7c | ±8 g cannot measure impacts; CRITICAL sits near full scale | Design limit | Documented; clipping flagged |
| 8 | Alerts have no hysteresis, hold, latch, or reset | High | Fixed by default (latch optional) |
| 9 | Unused globals, `String` heap churn, `tone()` portability | Low | Fixed |
| 10 | 400 kHz I2C with a PCF8574 LCD backpack | Low | Kept at 400 kHz, configurable |
| 11 | First LCD page is skipped immediately | Low | Fixed |
| 12 | Buzzer cooldown can swallow a more severe alert | Medium | Fixed |
| 13 | A missing or failed sensor is reported as a healthy zero | Medium | Fixed |
| 14 | Start-up PPG fill can wait forever | Medium | Fixed |
| 15 | Alert lock screen does not say what caused the alert | Low | Fixed |
| 16 | Log mixes free text with CSV | Low | Fixed |

---

## 1. Gravity compensation

**Evidence.** Line 402: `gz_comp = gz - 1.0f`, then the magnitude of `(gx, gy, gz_comp)` is called `gTotal`. This is zero at rest only if the sensor's Z axis points straight up. For a stationary helmet tilted by an angle θ the result is `2·sin(θ/2)` g: 0.26 g at 15°, 0.52 g at 30°, 1.41 g at 90°. A driver's head is rarely level and the mounting orientation is arbitrary. Lines 404–405 also take lateral and longitudinal g directly from the sensor X and Y axes, which is only right for one mounting orientation. The session peak (line 406) inherits the error permanently.

**Options considered.**

| Option | For | Against |
|---|---|---|
| Keep fixed Z | Simple | Wrong whenever the head is not level |
| Use `\|a\| − 1` (magnitude only) | Orientation-independent, no state | Badly under-reads horizontal loads: 1 g of braking reads as 0.41 g |
| Plain low-pass gravity estimate | Tracks orientation | A corner lasting longer than the filter time constant is absorbed into "gravity" and the reading decays to zero. Racing corners last seconds |
| **Gated low-pass (chosen)** | Tracks orientation at rest and on straights; freezes during sustained load | Cannot see orientation changes that happen during the load |
| Gyroscope + sensor fusion | Correct | Needs hardware the project does not have |

**Change.** `core::GravityEstimator` (`src/core/gravity.cpp`). The estimate starts from the mean reading over the 5 s calibration and then follows the reading with a 1 s time constant, but only while the reading's magnitude is within 0.15 g of 1 g. Dynamic acceleration is `reading − estimate`. Lateral, longitudinal and vertical components come from a frame built from the gravity direction and a configurable forward hint (`IMU_FORWARD_HINT_*`, default sensor Y, matching the original). Fixed by default; `IMU_GRAVITY_MODE = FixedSensorZ` restores the original.

**Why these numbers.** The largest error this method can make is the phantom from an unseen tilt, at most about 0.5 g for 30°, against a lowest alert threshold of 2.5 g. Accelerations below about 0.46 g sit inside the gate and are slowly absorbed (1 s time constant); that is the price of tracking head movement and is irrelevant to the alert thresholds. Both constants are untuned guesses and should be revisited with real data.

**Verify on hardware.**
1. Hold the helmet still in several orientations, including upside down. After a second or two `g_total` should settle below about 0.05 g in every one. With `FixedSensorZ` it will not.
2. Tilt it slowly through 90°: `g_total` should stay small throughout.
3. Swing it on a cord in a steady circle (sustained load): `g_total` should hold its value, not decay.
4. With the helmet worn in driving position, check that braking shows up in `g_long` and cornering in `g_lat`. If they are swapped or mixed, set the forward hint.

## 2. Blocking calls

**Evidence.**
- `buzzAlert()` (lines 453–454) calls `delay(150)` for DANGER and `delay(100)` twice for CRITICAL, from inside the 50 Hz alert path, at most once a second.
- `calibrateGSR()` (lines 382–386) blocks for 5 s.
- The start-up PPG fill (lines 229–234) blocks until 100 samples have arrived.

During any of these the PPG sensor is not serviced. The SparkFun driver keeps only 4 samples between `check()` and `nextSample()` (`STORAGE_SIZE 4` in `MAX30105.h`), which is 160 ms at 25 samples per second, so a 200 ms block loses samples outright, and the beat timer (which uses `millis()` at the moment of processing) sees distorted intervals.

**Change.** Nothing after `setup()` calls `delay()`.
- Buzzer patterns are a state machine, `core::ToneSequencer` (`src/core/buzzer_pattern.cpp`), with the same notes and timings as the original.
- Calibration runs inside the normal loop as a 5 s state while all sensors keep sampling (`src/main.cpp`).
- There is no PPG pre-fill: SpO2 simply appears once 100 samples have been collected.
- The remaining `delay()` calls are in `setup()` before sampling starts, to keep boot messages readable.

**Verify on hardware.** Trigger a CRITICAL alert (shake the sensor hard) while watching the log: the row spacing should stay at 100 ms throughout the alarm.

## 2a. `getIR()` and `getRed()` wait for new data

**Evidence.** Lines 313–315 (and 231–233 at start-up):

```cpp
uint32_t irValue  = ppgSensor.getIR();
uint32_t redValue = ppgSensor.getRed();
ppgSensor.nextSample();
```

In the SparkFun driver (v1.1.2, `MAX30105.cpp`), `getIR()` and `getRed()` each call `safeCheck(250)`, which loops with `delay(1)` until a *new* sample arrives, then return the newest sample. They are not the accessors for the sample that `available()` just reported; those are `getFIFOIR()` and `getFIFORed()`. (SparkFun's own SpO2 example uses the same pattern, which is presumably where it came from.)

At 25 samples per second this means, for each call to `readPPG()` that finds data:

- `getIR()` blocks for up to 40 ms waiting for the next sample;
- `getRed()` blocks for up to another 40 ms and returns red from a *later* sample than the IR value;
- so the "50 Hz" loop is held for about 80 ms most times round, i.e. it runs at roughly 12 Hz;
- the accelerometer, a 200 Hz device, is therefore read about 12 times a second;
- the SpO2 window receives about one sample per 80 ms, half the 25 Hz the Maxim algorithm assumes, with red and IR misaligned by one sample period;
- the start-up fill takes about 8 s, not 4 s.

**Change.** `src/ppg.cpp` drains the FIFO with `getFIFOIR()`/`getFIFORed()`/`nextSample()`, which never touch the bus or wait. Polling is limited to every 10 ms because `check()` itself costs several I2C transfers. If more samples arrive than the driver can hold, the overrun is detected, the SpO2 window is restarted and the beat timer re-referenced.

**Verify on hardware.** Flash the archived sketch and look at the `time_ms` column: the gaps should be irregular and around 80 ms or more instead of 100 ms sharp, if this analysis is right. Then flash this firmware and confirm steady 100 ms rows, and that `hr_maxim_bpm` roughly agrees with `hr_bpm` on a still fingertip (with the original data rate it would be expected to read about double).

## 3. `printf` format with a stray `%`

**Evidence.** Line 498: `snprintf(buf1, sizeof(buf1), "SpO2:  --- %   ");`. A `%` followed by spaces and the end of the string is an incomplete conversion specification, which is undefined behaviour. This is the line shown whenever there is no SpO2 reading.

The other format strings were checked for width; all fit in 16 characters plus the terminator for the value ranges they can receive:

| Line | Format | Longest output |
|---|---|---|
| 490 | `"HR: %3d bpm %s"` | 15 |
| 496 | `"SpO2:  %3d %%%s  "` | 15 |
| 512 | `"G:%4.1f Pk:%4.1f"` | 14 |
| 513 | `"Lat%4.1f %s"` | 13 |
| 519 | `"Stress:  %3d%%   "` | 16 |
| 520 | `"GSR raw: %4d   "` | 16 |

**Change.** All LCD text is produced by `src/core/lcd_format.cpp`. Every function returns exactly 16 characters for any input, including out-of-range, negative, NaN and infinite values, and this is unit-tested (`test/test_lcd_format`).

**Verify on hardware.** Take your finger off the pulse sensor; the second line should read `SpO2:  --- %`.

## 4. PPG sample rate and LED current

**Evidence.** Line 189: `ppgSensor.setup(60, 4, 2, 100, 411, 4096)`.

- 100 Hz conversion rate with 4× averaging gives **25 samples per second**. The Maxim algorithm in `spo2_algorithm.h` is written for exactly that (`#define FreqS 25`, `BUFFER_SIZE (FreqS * 4)` = 100). **The configuration is correct.**
- The comment on lines 305–306 ("a new reading roughly every 4 seconds at 100Hz") is wrong: the window is 4 s long and the result is refreshed every 25 samples, which is every 1 s. (In the original, because of issue 2a, it was in practice slower.)
- The comment on line 188 says "LED power 60mA". The first argument is a register value, not milliamps. On the MAX30102 each step is 0.2 mA, so 60 is about 12 mA. Lines 190–191 then overwrite both LEDs with `0x3F` (63, about 12.6 mA), so the 60 never takes effect.

**Change.** One constant, `PPG_LED_AMPLITUDE = 0x3F`, is passed to `setup()`; the effective LED current is unchanged. `src/ppg.cpp` has `static_assert`s tying the configured output rate and window to `FreqS` and `BUFFER_SIZE`, so a future change to the sensor settings that breaks the algorithm's assumption fails the build.

**Not verified.** The SparkFun beat detector (`checkForBeat`) has no stated sample rate; its example runs it at 100 samples per second. At 25 its low-pass filter cuts off four times lower in Hz, which may attenuate fast heart rates. This was true of the original too. Worth testing with a known fast pulse (after exercise) against a reference.

## 5. Heart-rate averaging

| | Evidence | Change |
|---|---|---|
| a | `rates[]` starts at zero (line 94) and the average always divides by 4 (lines 327–329), so the first reading at 60 bpm is reported as 15, then 30, 45, 60. Two of those are below `HR_LOW` and raise a false DANGER alert | Average over the beats actually stored |
| b | `(byte)beatsPerMinute` (line 324) truncates the fraction. Values are limited to below 220 so there is no overflow | Kept as float, average rounded |
| c | `lastBeat` starts at 0 (line 96), and after a no-contact period it still holds the time of the last beat before contact was lost. The first interval after either is measured from an unrelated moment; if it happens to fall in 20–220 bpm it is accepted | First beat after a reset only sets the reference |
| d | The no-contact reset (lines 366–372) runs *after* the same sample has been through beat detection and into the SpO2 window, and uses a single sample against a single threshold | Contact is decided first, with hysteresis (50000 on / 45000 off) and a 3-sample debounce; samples without contact go no further; the SpO2 window restarts when contact returns |
| e | The Maxim algorithm's heart rate is used once at start-up (line 240) and then computed every second and thrown away (lines 356–362). The display can therefore show a value from either source | One source for display and alerts (beat timing). The Maxim value is logged as `hr_maxim_bpm` so the two can be compared |
| f | Once set, heart rate and SpO2 never clear while there is contact. If beats stop being detected (motion, poor contact) the last value is shown indefinitely as if current | Heart rate blanks after 5 s without an accepted beat, SpO2 after 10 s without a valid result (`PPG_*_STALE_MS`, 0 restores the original) |
| g | Beat intervals are timed with `millis()` when the sample is processed, so they include however late the loop was | Intervals are counted in samples (40 ms each by the sensor's clock) |

All of this is in `core::BeatAverager` and `core::ContactDetector` (`src/core/ppg_logic.cpp`), unit-tested in `test/test_ppg_logic`.

A limitation that remains: at 25 samples per second a beat can only be placed to the nearest 40 ms. At 180 bpm (333 ms between beats) that is about ±12 % per beat before the four-beat average, and the HR_HIGH threshold is 180.

A library quirk noted but not changed: `averageDCEstimator()` in the SparkFun `heartRate.cpp` takes its sample as `uint16_t`, so 18-bit IR values are truncated on the way in. It still works because the later filter stage is also 16-bit and the offset cancels, but a signal that crosses a multiple of 65536 will glitch.

**Verify on hardware.** Place a finger: the first displayed value should already be plausible, with no false low-HR alert. Lift the finger: both values should blank within a quarter of a second. Compare `hr_bpm` with a reference (pulse oximeter or chest strap) at rest and after exercise.

## 6. GSR

### 6a. "Sensor OK" can never fail

**Evidence.** Lines 205–210: `if (testRead >= 0 && testRead <= 4095)`. `analogRead()` on the ESP32 always returns 0–4095, so `sensorOk_gsr` is always true. GPIO34 has no internal pull resistors, so an unconnected pin floats and returns arbitrary mid-range values.

**Change.** Readings at either ADC rail are treated as a fault (`GSR_RAIL_LOW`, `GSR_RAIL_HIGH`), the index is forced to 0 and the LCD shows `---`. Calibration fails with a logged warning if every reading was at a rail.

**What this does not solve.** A floating pin cannot be told apart from a connected sensor in software. **Recommended hardware fix:** a 1 MΩ resistor from GPIO34 to ground, so an unplugged sensor reads the low rail and is detected. The rail limits are provisional and need checking against real readings.

### 6b. ADC non-linearity

**Evidence.** The ESP32 ADC is non-linear, particularly near the ends of its range at 11 dB attenuation, and varies from chip to chip.

**Change.** `analogReadMilliVolts()` by default, which applies the factory calibration stored in the chip's eFuse. The log column is `gsr_mv`. `GSR_USE_MILLIVOLTS = false` restores raw counts. Attenuation is now set for the one pin, not globally.

### 6c. Polarity of the GSR signal (unverified)

**Evidence.** The original treats a *rise* in the reading as increased "stress" (lines 420–422).

Seeed's documentation for this sensor gives:

```
Human Resistance = ((1024 + 2 × reading) × 10000) / (calibration − reading)   ohms
```

where `calibration` is the reading with the electrodes not worn. For this to give a positive, finite resistance the worn reading must be *below* the calibration value, and the further below it is, the *lower* the resistance. Lower skin resistance is higher conductance, which is what arousal produces. Taken at face value, the formula says **the reading falls as arousal rises**, the opposite of what the sketch assumes. If so, the original index would stay at 0 during arousal and only rise as the driver relaxed.

However, the same page tells the user to adjust the potentiometer "until the serial output is minimized" with the sensor not worn, which does not sit comfortably with the formula. The documentation is not clear enough to settle it, and it has not been measured here.

**Change.** The polarity is one switch, `GSR_RISE_IS_AROUSAL`. It is left at the original value (`true`) because the evidence is documentary and ambiguous. The index function supports both directions and both are unit-tested.

**Verify on hardware (one minute).** Put the electrodes on, let the reading settle, and watch `gsr_mv`. Take a sharp deep breath, or have someone startle you. Within one to three seconds the reading will move one way and then recover. If it moved *down*, set `GSR_RISE_IS_AROUSAL = false`. As a cross-check, wetting the fingertips (lower resistance) must move it the same way.

### 6d. Fixed baseline

**Evidence.** The baseline is measured once (line 387) and never updated. Inside a helmet, sweat builds up steadily and the electrodes settle, so the reading drifts in one direction over a session and the index drifts with it.

**Change.** Optional slow baseline tracking, `GSR_BASELINE_TAU_S` (for example 120 s). Off by default, which is the original behaviour, because tracking changes what the index means: it becomes a measure of recent change, and a sustained state decays to zero. The serial command `c` reruns the calibration at any time.

### 6e. Naming and a division by zero

**Evidence.** The value is called a stress index and compared against "stress" thresholds. It is the rise of a voltage above its start-up value, as a fraction of the remaining ADC range. Line 421 divides by `(4095.0f − gsrBaseline)`, which is zero if the input was at full scale during calibration; the result is then converted to `int`, which is undefined for infinity or NaN.

**Change.** Renamed to *GSR arousal index* throughout (`gsr_index`, LCD `Arousal:`). `core::gsrIndex()` returns 0 when the baseline leaves less than 1 % of full scale as headroom. The thresholds (60 / 80) are unchanged.

## 7. Accelerometer

### 7a. The ±8 g range is really ±4 g

**Evidence.** Line 172 selects `MMA8451_RANGE_8_G`. But `Adafruit_MMA8451::begin()` (v1.2.3) ends with:

```cpp
// Activate at max rate, low noise mode
writeRegister8(MMA8451_REG_CTRL_REG1, 0x01 | 0x04);
```

Bit 2 of `CTRL_REG1` is `LNOISE`. `setRange()` and `setDataRate()` both read the register back and preserve that bit. The MMA8451Q datasheet says of it: "In Low Noise mode, the maximum signal that can be measured is ±4g. Note: Any thresholds set above 4g will not be reached."

So with the original code no single axis can read beyond ±4 g. The DANGER threshold (4.0 g) is at the clipping point and CRITICAL (6.0 g) can only be reached when two or three axes are loaded at once (the most the vector can reach is 4·√3 ≈ 6.9 g).

**Change.** `src/imu.cpp` rewrites `CTRL_REG1` after configuration with `LNOISE` cleared (`IMU_CLEAR_LOW_NOISE`, on by default). The cost is slightly higher noise (the datasheet gives 126 against 99 µg/√Hz), which does not matter at these thresholds.

**Verify on hardware.** Shake the sensor hard along one axis and look at the raw axis values (temporarily print `mma.x_g`). With the fix they should reach about ±8; with `IMU_CLEAR_LOW_NOISE = false` they should flatten at about ±4.

### 7b. Peaks between polls are missed

**Evidence.** The sensor produces 200 samples per second; `readAccelerometer()` is scheduled at 50 Hz (line 278) and, because of issue 2a, actually ran at roughly 12 Hz. Each read returns only the latest sample. A bump lasting 10–20 ms will usually fall between reads, so both the alert and the session peak miss it.

**Options considered.** (1) Poll at the sensor rate. (2) Use the MMA8451's 32-sample FIFO and read it in batches. (3) Use the sensor's transient-detection interrupt. Option 1 was chosen because it needs no driver changes and the loop is now fast enough; 2 and 3 are better still and are listed as future work, because the Adafruit driver exposes neither.

**Change.** `IMU_POLL_HZ = 200`. The alert logic takes the largest value since its previous run (`imu::takeAlertPeak()`), and the log has a `g_peak_window` column with the largest value since the previous row, so a short peak appears in the log even at 10 rows per second.

One known gap: an LCD refresh occupies the I2C bus briefly, during which an accelerometer sample can be skipped. The display now sends only the characters that changed to keep this short. A FIFO read would close the gap completely.

### 7c. Range against the use case

±8 g covers cornering and braking but is one or two orders of magnitude below head accelerations in a crash; the sensor will clip. That cannot be fixed in software. Clipped samples are detected (any axis within 2 % of full scale) and flagged in the log `status` column, so a clipped peak can at least be recognised as a lower bound. A high-g part such as the ADXL375 (±200 g) is listed as future work.

## 8. Alerts

**Evidence.** `evaluateAlerts()` (lines 428–442) recomputes the level from scratch every 20 ms and overwrites `currentAlert` and `alertLocked`.

- A spike that is over by the next tick gives CRITICAL for 20 ms. The LCD redraws every 200 ms and the log writes every 100 ms, so the display will usually never show it and the log has about a one-in-five chance of recording it. The "lock to the g-force page" releases at once.
- A value hovering at a threshold switches the alert on and off continuously.
- `gPeakSession` can never be reset without a reboot.
- There is no way to acknowledge anything.

**Change.** Two small classes in `src/core/alerts.cpp`:

- `AlertEvaluator` applies the original thresholds with hysteresis (`G_HYSTERESIS` 0.3 g, `GSR_INDEX_HYSTERESIS` 5, `HR_HYSTERESIS` 5 bpm). With all three at 0 it is exactly the original rule; a unit test checks that.
- `AlertLatch` keeps a level up for at least `ALERT_HOLD_MS` (3 s) after its cause has gone, and can optionally latch CRITICAL until acknowledged (`ALERT_LATCH_CRITICAL`, off by default).

The log records both the held level (`alert`) and the instantaneous one (`alert_instant`), plus which measurements are responsible (`alert_sources`).

**Acknowledge and reset.** The hardware has no button. Serial commands are provided (`a` acknowledge, `r` reset session peak, `c` recalibrate), and `PIN_ACK_BUTTON` enables an optional active-low button; GPIO0, the BOOT button on most dev boards, would work with no extra parts. It is disabled by default.

Hysteresis and hold are on by default because without them the alert display does not do its job. Setting `ALERT_HOLD_MS` and the three hysteresis constants to 0 restores the original behaviour.

**Verify on hardware.** Tap the sensor sharply: the alert should stay on the LCD for 3 s and appear in several log rows. Hold the GSR index or g-force near a threshold: the level should not chatter.

## 9. Smaller code issues

| Evidence | Change |
|---|---|
| `bufferIndex`, `bufferFull` (lines 89–90) and `lastSpo2Calc` (line 123) are never used | Removed |
| `lcdPrintPadded()` (lines 549–553) builds three `String` objects per line, 10 lines a second, on the heap | Fixed-size `char` buffers; only changed characters are written |
| `tone()` (lines 260–262, 452–454): present in the pinned core (arduino-esp32 2.0.17, `cores/esp32/Tone.cpp`), where it is non-blocking and queues notes, so the original's `delay()` calls were for spacing. Older ESP32 cores did not provide it | The buzzer uses LEDC PWM directly (`src/buzzer.cpp`), with a compile-time branch for the changed LEDC API in core 3.x |
| `long lastBeat` with `millis()` (lines 96, 319): signed arithmetic on an unsigned counter | Unsigned throughout; wrap-around is unit-tested |
| `switch (lcdPage)` (line 481) has no default; the buffers would be printed uninitialised if the page were ever out of range | Default case |

## 10. I2C at 400 kHz

**Evidence.** Line 153 sets 400 kHz, and `ppgSensor.begin(Wire, I2C_SPEED_FAST)` (line 186) sets it again. The MMA8451 and MAX30102 both support 400 kHz. The LCD backpack at address 0x3F is almost certainly a PCF8574A, which is specified for 100 kHz. In practice most of these boards work at 400 kHz, which is presumably why the original did.

**Change.** Kept at 400 kHz and made a single constant, `I2C_CLOCK_HZ`. The MAX30102 driver changes the bus speed inside `begin()`, so the configured speed is reapplied afterwards.

At 100 kHz everything still fits, but with less margin: the accelerometer is read 200 times a second and the driver uses two transfers per read.

**Verify on hardware.** If the LCD ever shows wrong characters, set `I2C_CLOCK_HZ = 100000` first.

Related: the display is now probed at start-up. If nothing answers at the configured address a warning is logged and all LCD writes are skipped, so a missing or wrongly addressed display cannot slow the loop with failed transfers.

## 11. The first LCD page is skipped

**Evidence.** `lcdPage` starts at 1 "starts on HR/SpO2 page" (line 126) and `lastPageFlip` at 0 (line 127). By the time `updateLCD()` first runs, more than 3 s have passed since boot, so line 472 is immediately true and the page advances to 2 before page 1 is ever drawn.

**Change.** The rotation timer starts when the pages first appear (`src/display.cpp`).

## 12. The buzzer cooldown can swallow a worse alert

**Evidence.** Line 449: any alert within 1 s of the previous pattern is silently dropped, whatever its level. A WARNING beep followed 300 ms later by a brief CRITICAL spike produces no CRITICAL sound at all.

**Change.** `core::AlertBuzzer` lets an alert more severe than the one last sounded start straight away. The same or a lower level still waits for the cooldown.

## 13. Sensor failures look like healthy zeros

**Evidence.** If the MMA8451 is not found, `readAccelerometer()` returns early (line 396) and the display shows `G: 0.0 ... OK` for the whole session. The same applies to the pulse sensor. A sensor that stops responding after start-up is not detected at all; the Adafruit driver then returns the same value on every read.

**Change.** Missing sensors are shown as `no sensor` and readings without a value as `---`. The accelerometer is marked failed after 100 identical consecutive raw samples (a live 14-bit accelerometer always shows noise), and the pulse sensor after 1 s without any samples; both recover on their own. The state of each sensor is in the log `status` column. Sensor faults do not currently raise an alert; that is listed as future work.

## 14. Start-up fill can wait forever

**Evidence.** Line 230: `while (!ppgSensor.available()) ppgSensor.check();` has no timeout. If the sensor stops delivering samples after a successful `begin()`, the device stays on "Place finger on PPG sensor..." permanently.

**Change.** There is no blocking fill (see issue 2).

## 15. The alert lock screen does not show the cause

**Evidence.** On DANGER or CRITICAL the display locks to the g-force page (line 477) even when the cause is heart rate or GSR. The driver sees `DANGR` beside a g-force of 0.3 with no explanation.

**Change.** The alert line ends with the cause: `G`, `S` (skin), `H` (heart), for example `Lat 0.3 DNGR H`. The level labels are now four characters (`OK`, `WARN`, `DNGR`, `CRIT`) to make room.

## 16. Log format

**Evidence.** The banner, sensor status and CSV header are printed as plain text in the same stream as the data (lines 556–565), so a parser has to guess where the data starts.

**Change.** Every non-data line starts with `#`. The header is printed once. The first ten columns keep the original order, and six columns are appended (see the README).

---

## Behaviour switches

All in `src/config.h`. "Original" is the value that reproduces the sketch.

| Switch | Default | Original |
|---|---|---|
| `IMU_GRAVITY_MODE` | `GatedLowPass` | `FixedSensorZ` |
| `IMU_CLEAR_LOW_NOISE` | `true` | `false` |
| `IMU_POLL_HZ` | 200 | 50 |
| `PPG_IR_ABSENT` / `PPG_CONTACT_DEBOUNCE` | 45000 / 3 | 50000 / 1 |
| `PPG_HR_STALE_MS` / `PPG_SPO2_STALE_MS` | 5000 / 10000 | 0 / 0 |
| `GSR_USE_MILLIVOLTS` | `true` | `false` |
| `GSR_RISE_IS_AROUSAL` | `true` | `true` |
| `GSR_BASELINE_TAU_S` | 0 (off) | 0 |
| `G_HYSTERESIS` / `GSR_INDEX_HYSTERESIS` / `HR_HYSTERESIS` | 0.3 / 5 / 5 | 0 / 0 / 0 |
| `ALERT_HOLD_MS` | 3000 | 0 |
| `ALERT_LATCH_CRITICAL` | `false` | `false` |
| `PIN_ACK_BUTTON` | -1 (none) | -1 |
| `I2C_CLOCK_HZ` | 400000 | 400000 |

Changes with no switch, because the original behaviour was a defect: the `printf` fix, non-blocking buzzer and calibration, correct FIFO accessors, heart-rate averaging fixes, the division-by-zero guard, first-page fix, cooldown escalation, and sensor-fault display.

## Open questions

1. **GSR polarity** (6c). Needs the one-minute bench test.
2. **Which LCD is it?** The wiring table says "Adafruit LCD" on 3V3 at 0x3F. Address 0x3F and the `LiquidCrystal_I2C` library both point to a generic PCF8574A backpack, not an Adafruit one. Most such displays need 5 V for usable contrast; if it is powered from 5 V, check whether the backpack's pull-ups put 5 V on SDA and SCL, which the ESP32 does not tolerate.
3. **What are the g thresholds meant for?** 2.5, 4 and 6 g are within the normal cornering and braking range of a high-downforce racing car, so in that setting the alarm would sound on most corners. They are more plausible for karting or as impact indicators. The intent should be stated.
4. **Should a GSR index of 80 really be a DANGER, the same level as 4 g?** Inside a helmet, sweat alone will raise skin conductance.
5. **Where is the optical sensor worn?** The contact thresholds and LED current are fingertip values.
6. **Is SpO2 meant to alert?** The display flags values below 94 % but no alert is raised. That has been left as it was.
7. **Who reads the LCD?** The driver cannot see a display on their own helmet; if it is for a pit crew or bench demo, that affects what the pages should show.

## Bench checklist

- [ ] `pio test -e native` passes on a machine with a host compiler
- [ ] Boots, calibrates, prints the header and steady 100 ms rows
- [ ] `g_total` near zero at rest in every orientation (issue 1)
- [ ] Single-axis readings exceed 4 g when shaken hard (issue 7a)
- [ ] A sharp tap gives an alert that is visible for 3 s and present in the log (issues 7b, 8)
- [ ] Buzzer patterns sound as before; log rows keep their timing during an alarm (issue 2)
- [ ] Heart rate plausible from the first reading, blanks when contact is lost, agrees with a reference (issue 5)
- [ ] `hr_maxim_bpm` roughly agrees with `hr_bpm` on a still fingertip (issues 2a, 4)
- [ ] GSR direction test done and `GSR_RISE_IS_AROUSAL` set accordingly (issue 6c)
- [ ] GSR unplugged is detected, after fitting the pull-down (issue 6a)
- [ ] LCD is clean at 400 kHz, or `I2C_CLOCK_HZ` lowered (issue 10)
- [ ] Unplugging a sensor mid-run shows `no sensor` and clears the matching `status` bit (issue 13)
