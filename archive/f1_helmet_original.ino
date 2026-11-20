// HISTORICAL REFERENCE ONLY: the original single-file sketch this project grew from; not built, kept unmodified apart from this line and the restored opening "/*".

/* ============================================================
 *  F1 HELMET DRIVER SENSOR SYSTEM
 *  Target board : ESP32 Dev Module (NodeMCU-32S)
 * ============================================================
 *
 *  REQUIRED LIBRARIES
 *  ─────────────────────────────────────────────────────────────
 *  1. LiquidCrystal_I2C       — Library Manager (Frank de Brabander)
 *  2. Adafruit MMA8451        — Library Manager (Adafruit)
 *  3. Adafruit Unified Sensor — Library Manager (Adafruit)
 *  4. SparkFun MAX3010x       — Library Manager (SparkFun Electronics)
 *     Search: "SparkFun MAX3010x Pulse and Proximity Sensor Library"
 *  5. Wire                    — Built-in, no install needed
 *
 *  NOTE: DFRobot_BloodOxygen_S and DFRobot_RTU are no longer needed.
 *
 *  WIRING  (NodeMCU-32S)
 *  ─────────────────────────────────────────────────────────────
 *  Component         VCC    GND    SDA    SCL    Other
 *  ──────────────    ───    ───    ───    ───    ─────
 *  Adafruit LCD      3V3    GND    21     22     –
 *  Adafruit MMA8451  3V3    GND    21     22     –
 *  MAX30102          3V3    GND    21     22     –
 *  GSR V1.2          3V3    GND    –      –      SIG→GPIO34
 *  Piezo Buzzer      –      GND    –      –      +→GPIO25
 *
 *  I2C Addresses
 *  ─────────────────────────────────────────────────────────────
 *  LCD      : 0x3F  (change to 0x27 if display stays blank)
 *  MMA8451  : 0x1D
 *  MAX30102 : 0x57  (SparkFun library handles this automatically)
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"           // SparkFun MAX3010x library
#include "heartRate.h"          // SparkFun beat detection algorithm
#include "spo2_algorithm.h"     // SparkFun SpO2 algorithm

// ── Pin Definitions ──────────────────────────────────────────
#define GSR_PIN        34
#define BUZZER_PIN     25

// ── I2C Addresses ────────────────────────────────────────────
#define LCD_I2C_ADDR   0x3F    // Change to 0x27 if LCD stays blank

// ── Tuning Constants ─────────────────────────────────────────
#define GSR_SAMPLES         8
#define GSR_CALIBRATION_MS  5000
#define SENSOR_READ_HZ      50
#define LCD_REFRESH_HZ      5
#define SERIAL_LOG_HZ       10

// SpO2 / HR buffer size — SparkFun algorithm needs 100 samples
#define SPO2_BUFFER_SIZE    100

// G-force alert thresholds
#define G_WARN      2.5f
#define G_DANGER    4.0f
#define G_CRITICAL  6.0f

// Stress index alert thresholds (0–100)
#define STRESS_WARN    60
#define STRESS_DANGER  80

// Heart rate alert thresholds (BPM)
#define HR_LOW   40
#define HR_HIGH  180

#define BUZZER_COOLDOWN_MS  1000

// ── Sensor Objects ───────────────────────────────────────────
LiquidCrystal_I2C  lcd(LCD_I2C_ADDR, 16, 2);
Adafruit_MMA8451   mma;
MAX30105           ppgSensor;

// ── SpO2 / HR Buffers (SparkFun algorithm) ───────────────────
uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];
int32_t  spo2Value      = 0;
int8_t   spo2Valid      = 0;
int32_t  heartRateValue = 0;
int8_t   hrValid        = 0;
uint8_t  bufferIndex    = 0;
bool     bufferFull     = false;

// Beat detection (for live HR flash on LCD)
const byte RATE_SIZE = 4;
byte   rates[RATE_SIZE];
byte   rateSpot   = 0;
long   lastBeat   = 0;
float  beatsPerMinute = 0;
int    beatAvg    = 0;

// ── Sensor Data ──────────────────────────────────────────────
struct SensorData {
  float gTotal       = 0.0f;
  float gLateral     = 0.0f;
  float gLong        = 0.0f;
  float gPeakSession = 0.0f;
  int   stressIndex  = 0;
  int   gsrRaw       = 0;
  int   heartRate    = 0;
  int   spo2         = 0;
  bool  sensorOk_mma = false;
  bool  sensorOk_ppg = false;
  bool  sensorOk_gsr = false;
};

SensorData data;
float gsrBaseline = 2048.0f;

// Timing
unsigned long lastSensorRead = 0;
unsigned long lastLCDUpdate  = 0;
unsigned long lastSerialLog  = 0;
unsigned long lastBuzzer     = 0;
unsigned long lastSpo2Calc   = 0;

// LCD paging — starts on HR/SpO2 page
uint8_t       lcdPage      = 1;
unsigned long lastPageFlip = 0;
bool          alertLocked  = false;

enum AlertLevel { ALERT_NONE, ALERT_WARNING, ALERT_DANGER, ALERT_CRITICAL };
AlertLevel currentAlert = ALERT_NONE;

// ── Forward Declarations ─────────────────────────────────────
void calibrateGSR();
void readAccelerometer();
void readGSR();
void readPPG();
void evaluateAlerts();
void buzzAlert(AlertLevel level);
void updateLCD();
void logSerial();
void lcdPrintPadded(const String& str, uint8_t maxLen = 16);
void printStartupInfo();

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  // ── Buzzer ───────────────────────────────────────────────
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // ── LCD ──────────────────────────────────────────────────
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("F1 Helmet System");
  lcd.setCursor(0, 1);
  lcd.print(" Initialising.. ");
  delay(800);

  // ── MMA8451 Accelerometer ────────────────────────────────
  lcd.setCursor(0, 1);
  lcd.print(" MMA8451...     ");
  if (mma.begin(0x1D)) {
    mma.setRange(MMA8451_RANGE_8_G);
    mma.setDataRate(MMA8451_DATARATE_200_HZ);
    data.sensorOk_mma = true;
    Serial.println("[OK] MMA8451 ready");
  } else {
    Serial.println("[ERR] MMA8451 not found");
    lcd.setCursor(0, 1);
    lcd.print(" MMA FAIL!      ");
    delay(1500);
  }

  // ── MAX30102 PPG (SparkFun library) ──────────────────────
  lcd.setCursor(0, 1);
  lcd.print(" MAX30102...    ");
  if (ppgSensor.begin(Wire, I2C_SPEED_FAST)) {
    // Configure sensor:
    // Sample rate 100Hz, pulse width 411us, ADC range 4096, LED power 60mA
    ppgSensor.setup(60, 4, 2, 100, 411, 4096);
    ppgSensor.setPulseAmplitudeRed(0x3F);   // Red LED power for SpO2
    ppgSensor.setPulseAmplitudeIR(0x3F);    // IR LED power for SpO2
    data.sensorOk_ppg = true;
    Serial.println("[OK] MAX30102 ready (SparkFun driver)");
  } else {
    Serial.println("[ERR] MAX30102 not found");
    lcd.setCursor(0, 1);
    lcd.print(" PPG FAIL!      ");
    delay(1500);
  }

  // ── GSR ──────────────────────────────────────────────────
  lcd.setCursor(0, 1);
  lcd.print(" GSR...         ");
  analogSetAttenuation(ADC_11db);
  int testRead = analogRead(GSR_PIN);
  if (testRead >= 0 && testRead <= 4095) {
    data.sensorOk_gsr = true;
    Serial.print("[OK] GSR on GPIO34, test: ");
    Serial.println(testRead);
  }

  // ── GSR Calibration ──────────────────────────────────────
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("GSR Calibrating ");
  lcd.setCursor(0, 1);
  lcd.print("Stay still 5s.. ");
  calibrateGSR();

  // ── Fill initial SpO2 buffer before starting main loop ───
  if (data.sensorOk_ppg) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Place finger on ");
    lcd.setCursor(0, 1);
    lcd.print("PPG sensor...   ");
    Serial.println("[INFO] Filling PPG buffer (100 samples)...");

    for (byte i = 0; i < SPO2_BUFFER_SIZE; i++) {
      while (!ppgSensor.available()) ppgSensor.check();
      redBuffer[i] = ppgSensor.getRed();
      irBuffer[i]  = ppgSensor.getIR();
      ppgSensor.nextSample();
    }
    // First SpO2 + HR calculation
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, SPO2_BUFFER_SIZE, redBuffer,
      &spo2Value, &spo2Valid, &heartRateValue, &hrValid
    );
    if (hrValid  && heartRateValue > 20 && heartRateValue < 220) data.heartRate = heartRateValue;
    if (spo2Valid && spo2Value > 50 && spo2Value <= 100)          data.spo2      = spo2Value;
    Serial.println("[INFO] PPG buffer filled, initial reading done");
  }

  // ── Ready ─────────────────────────────────────────────────
  printStartupInfo();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!   ");
  lcd.setCursor(0, 1);
  lcd.print("A:");
  lcd.print(data.sensorOk_mma ? "OK" : "--");
  lcd.print(" P:");
  lcd.print(data.sensorOk_ppg ? "OK" : "--");
  lcd.print(" G:");
  lcd.print(data.sensorOk_gsr ? "OK" : "--");
  delay(2000);
  lcd.clear();

  tone(BUZZER_PIN, 1000, 80);
  delay(120);
  tone(BUZZER_PIN, 1400, 80);
  delay(200);
}

// ════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── PPG: read every loop for smooth beat detection ───────
  if (data.sensorOk_ppg) {
    readPPG();
  }

  // ── 50 Hz: Accel + GSR + alerts ─────────────────────────
  if (now - lastSensorRead >= (1000 / SENSOR_READ_HZ)) {
    lastSensorRead = now;
    readAccelerometer();
    readGSR();
    evaluateAlerts();
  }

  // ── 5 Hz: LCD ────────────────────────────────────────────
  if (now - lastLCDUpdate >= (1000 / LCD_REFRESH_HZ)) {
    lastLCDUpdate = now;
    updateLCD();
  }

  // ── 10 Hz: Serial CSV ────────────────────────────────────
  if (now - lastSerialLog >= (1000 / SERIAL_LOG_HZ)) {
    lastSerialLog = now;
    logSerial();
  }
}

// ════════════════════════════════════════════════════════════
//  READ PPG (SparkFun MAX3010x)
//
//  Uses a rolling buffer approach:
//  - Dumps the oldest 25 samples from the buffer
//  - Reads 25 fresh samples into their place
//  - Recalculates HR and SpO2 on the full 100-sample window
//  This gives a new reading roughly every 4 seconds at 100Hz
//  but beat detection gives instant per-beat HR feedback.
// ════════════════════════════════════════════════════════════
void readPPG() {
  // Check if new data is available from the sensor
  ppgSensor.check();
  if (!ppgSensor.available()) return;

  uint32_t irValue  = ppgSensor.getIR();
  uint32_t redValue = ppgSensor.getRed();
  ppgSensor.nextSample();

  // ── Beat detection for instant HR feedback ───────────────
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat   = millis();
    beatsPerMinute = 60.0f / (delta / 1000.0f);

    if (beatsPerMinute > 20 && beatsPerMinute < 220) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      // Average the last 4 beats for a stable reading
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
      data.heartRate = beatAvg;
    }
  }

  // ── Rolling SpO2 recalculation every 25 new samples ──────
  // Shift buffer: drop 25 old samples, add 25 new ones
  static byte newSamples = 0;
  static uint32_t newIR[25], newRed[25];

  newIR[newSamples]  = irValue;
  newRed[newSamples] = redValue;
  newSamples++;

  if (newSamples >= 25) {
    newSamples = 0;
    // Shift old data down by 25
    for (byte i = 25; i < SPO2_BUFFER_SIZE; i++) {
      irBuffer[i - 25]  = irBuffer[i];
      redBuffer[i - 25] = redBuffer[i];
    }
    // Copy new samples into the top 25 slots
    for (byte i = 0; i < 25; i++) {
      irBuffer[75 + i]  = newIR[i];
      redBuffer[75 + i] = newRed[i];
    }
    // Recalculate SpO2 on the full 100-sample window
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, SPO2_BUFFER_SIZE, redBuffer,
      &spo2Value, &spo2Valid, &heartRateValue, &hrValid
    );
    if (spo2Valid && spo2Value > 50 && spo2Value <= 100) {
      data.spo2 = spo2Value;
    }
  }

  // If IR is very low, no finger is on the sensor — reset
  if (irValue < 50000) {
    data.heartRate = 0;
    data.spo2      = 0;
    beatAvg        = 0;
    beatsPerMinute = 0;
    memset(rates, 0, sizeof(rates));
  }
}

// ════════════════════════════════════════════════════════════
//  GSR CALIBRATION
// ════════════════════════════════════════════════════════════
void calibrateGSR() {
  long sum = 0;
  int  samples = 0;
  unsigned long start = millis();
  while (millis() - start < (unsigned long)GSR_CALIBRATION_MS) {
    sum += analogRead(GSR_PIN);
    samples++;
    delay(20);
  }
  gsrBaseline = (samples > 0) ? (float)sum / (float)samples : 2048.0f;
  Serial.print("[INFO] GSR baseline: ");
  Serial.println(gsrBaseline, 1);
}

// ════════════════════════════════════════════════════════════
//  READ ACCELEROMETER
// ════════════════════════════════════════════════════════════
void readAccelerometer() {
  if (!data.sensorOk_mma) return;
  sensors_event_t event;
  mma.getEvent(&event);
  float gx = event.acceleration.x / 9.80665f;
  float gy = event.acceleration.y / 9.80665f;
  float gz = event.acceleration.z / 9.80665f;
  float gz_comp = gz - 1.0f;
  data.gTotal   = sqrt(gx * gx + gy * gy + gz_comp * gz_comp);
  data.gLateral = fabs(gx);
  data.gLong    = fabs(gy);
  if (data.gTotal > data.gPeakSession) data.gPeakSession = data.gTotal;
}

// ════════════════════════════════════════════════════════════
//  READ GSR
// ════════════════════════════════════════════════════════════
void readGSR() {
  if (!data.sensorOk_gsr) return;
  long sum = 0;
  for (int i = 0; i < GSR_SAMPLES; i++) {
    sum += analogRead(GSR_PIN);
    delayMicroseconds(200);
  }
  data.gsrRaw = (int)(sum / GSR_SAMPLES);
  float delta      = (float)(data.gsrRaw - gsrBaseline);
  float normalised = constrain(delta / (4095.0f - gsrBaseline), -0.2f, 1.0f);
  data.stressIndex = (int)constrain(normalised * 100.0f, 0.0f, 100.0f);
}

// ════════════════════════════════════════════════════════════
//  EVALUATE ALERTS
// ════════════════════════════════════════════════════════════
void evaluateAlerts() {
  AlertLevel newAlert = ALERT_NONE;

  if      (data.gTotal >= G_CRITICAL)                                      newAlert = ALERT_CRITICAL;
  else if (data.gTotal >= G_DANGER  || data.stressIndex >= STRESS_DANGER)  newAlert = ALERT_DANGER;
  else if (data.gTotal >= G_WARN    || data.stressIndex >= STRESS_WARN)    newAlert = ALERT_WARNING;

  if (data.heartRate > 0 && (data.heartRate > HR_HIGH || data.heartRate < HR_LOW)) {
    if (newAlert < ALERT_DANGER) newAlert = ALERT_DANGER;
  }

  currentAlert = newAlert;
  alertLocked  = (newAlert >= ALERT_DANGER);
  if (newAlert != ALERT_NONE) buzzAlert(newAlert);
}

// ════════════════════════════════════════════════════════════
//  BUZZER PATTERNS
// ════════════════════════════════════════════════════════════
void buzzAlert(AlertLevel level) {
  unsigned long now = millis();
  if (now - lastBuzzer < BUZZER_COOLDOWN_MS) return;
  lastBuzzer = now;
  switch (level) {
    case ALERT_WARNING:  tone(BUZZER_PIN, 800,  150); break;
    case ALERT_DANGER:   tone(BUZZER_PIN, 1200, 100); delay(150); tone(BUZZER_PIN, 1200, 100); break;
    case ALERT_CRITICAL: tone(BUZZER_PIN, 2000, 80);  delay(100); tone(BUZZER_PIN, 2000, 80); delay(100); tone(BUZZER_PIN, 2000, 80); break;
    default: break;
  }
}

// ════════════════════════════════════════════════════════════
//  LCD UPDATE
//
//  Page 1 (default): Heart rate + SpO2
//  Page 0          : G-force + alert status
//  Page 2          : Stress + GSR raw
//
//  Rotates every 3 seconds.
//  Locks to page 0 on DANGER or CRITICAL alert.
// ════════════════════════════════════════════════════════════
void updateLCD() {
  unsigned long now = millis();

  if (!alertLocked && (now - lastPageFlip >= 3000)) {
    lcdPage = (lcdPage + 1) % 3;
    lastPageFlip = now;
  }

  if (alertLocked) lcdPage = 0;

  char buf0[17], buf1[17];

  switch (lcdPage) {

    // ── Page 1: HR + SpO2 ────────────────────────────────
    case 1: {
      const char* hrStatus = "   ";
      if      (data.heartRate > HR_HIGH) hrStatus = "!HI";
      else if (data.heartRate > 0 && data.heartRate < HR_LOW) hrStatus = "!LO";

      if (data.heartRate > 0)
        snprintf(buf0, sizeof(buf0), "HR: %3d bpm %s", data.heartRate, hrStatus);
      else
        snprintf(buf0, sizeof(buf0), "HR: --- bpm    ");

      if (data.spo2 > 0) {
        const char* o2warn = (data.spo2 < 94) ? "!" : " ";
        snprintf(buf1, sizeof(buf1), "SpO2:  %3d %%%s  ", data.spo2, o2warn);
      } else {
        snprintf(buf1, sizeof(buf1), "SpO2:  --- %   ");
      }
      break;
    }

    // ── Page 0: G-force + alert status ───────────────────
    case 0: {
      const char* st;
      switch (currentAlert) {
        case ALERT_CRITICAL: st = "CRIT!"; break;
        case ALERT_DANGER:   st = "DANGR"; break;
        case ALERT_WARNING:  st = "WARN "; break;
        default:             st = "OK   "; break;
      }
      snprintf(buf0, sizeof(buf0), "G:%4.1f Pk:%4.1f", data.gTotal, data.gPeakSession);
      snprintf(buf1, sizeof(buf1), "Lat%4.1f %s",      data.gLateral, st);
      break;
    }

    // ── Page 2: Stress + GSR ─────────────────────────────
    case 2: {
      snprintf(buf0, sizeof(buf0), "Stress:  %3d%%   ", data.stressIndex);
      snprintf(buf1, sizeof(buf1), "GSR raw: %4d   ",  data.gsrRaw);
      break;
    }
  }

  lcd.setCursor(0, 0); lcdPrintPadded(String(buf0));
  lcd.setCursor(0, 1); lcdPrintPadded(String(buf1));
}

// ════════════════════════════════════════════════════════════
//  SERIAL CSV LOG
//  time_ms, gTotal, gLat, gLong, gPeak, stress, gsrRaw, hr, spo2, alert
// ════════════════════════════════════════════════════════════
void logSerial() {
  Serial.print(millis());              Serial.print(',');
  Serial.print(data.gTotal,       2);  Serial.print(',');
  Serial.print(data.gLateral,     2);  Serial.print(',');
  Serial.print(data.gLong,        2);  Serial.print(',');
  Serial.print(data.gPeakSession, 2);  Serial.print(',');
  Serial.print(data.stressIndex);      Serial.print(',');
  Serial.print(data.gsrRaw);           Serial.print(',');
  Serial.print(data.heartRate);        Serial.print(',');
  Serial.print(data.spo2);             Serial.print(',');
  Serial.println((int)currentAlert);
}

// ════════════════════════════════════════════════════════════
//  HELPERS
// ════════════════════════════════════════════════════════════
void lcdPrintPadded(const String& str, uint8_t maxLen) {
  String padded = str;
  while ((int)padded.length() < maxLen) padded += ' ';
  lcd.print(padded.substring(0, maxLen));
}

void printStartupInfo() {
  Serial.println("========================================");
  Serial.println("  F1 Helmet Driver Sensor System");
  Serial.println("========================================");
  Serial.print("  MMA8451  : "); Serial.println(data.sensorOk_mma ? "OK" : "FAIL");
  Serial.print("  MAX30102 : "); Serial.println(data.sensorOk_ppg ? "OK" : "FAIL");
  Serial.print("  GSR V1.2 : "); Serial.println(data.sensorOk_gsr ? "OK" : "FAIL");
  Serial.print("  GSR base : "); Serial.println(gsrBaseline, 1);
  Serial.println("CSV: time_ms,gTotal,gLat,gLong,gPeak,stress,gsrRaw,hr,spo2,alert");
  Serial.println("========================================");
}
