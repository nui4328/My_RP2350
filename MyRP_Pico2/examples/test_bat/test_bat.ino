#include <Wire.h>

// ── ADS1115 & PCF8574 ────────────────────────────────────────────
#define ADS_ADDR          0x48
#define PCF_ADDR          0x20

// Continuous mode, ±4.096V, 250 SPS
const uint16_t ADC_CONFIG = 0x84C3;

// Calibration (ปรับตามจริงของคุณ)
const float CAL_SCALE  = 4.0296f;
const float CAL_OFFSET = 2.6699f;

// Buzzer (passive)
#define BUZZER_PIN 9

// แรงดันหลัก
const float V_FULL   = 12.40f;
const float V_EMPTY  = 11.11f;          // ต่ำกว่านี้ = LOW
const float V_MIN_VALID = 6.0f;         // ต่ำกว่านี้ = ไม่มีแบต → กระพริบแต่ไม่เสียง
const float V_STEP   = (V_FULL - V_EMPTY) / 7.0f;

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();

  // ตั้งค่า buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  Serial.println("\n=== Battery Monitor Started ===");
  Serial.printf("V_FULL     = %.3f V\n", V_FULL);
  Serial.printf("V_EMPTY    = %.3f V\n", V_EMPTY);
  Serial.printf("V_MIN_VALID = %.1f V (ต่ำกว่านี้ไม่เตือนเสียง)\n", V_MIN_VALID);
  Serial.println("กำลังเริ่มต้น ADS1115...");

  // ตั้งค่า ADS1115 continuous mode
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(0x01);
  Wire.write(ADC_CONFIG >> 8);
  Wire.write(ADC_CONFIG & 0xFF);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: ADS1115 config failed!");
    while (true) delay(1000);
  }

  delay(50);
  Wire.beginTransmission(ADS_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: ADS1115 not found!");
    while (true) delay(1000);
  }
  Serial.println("ADS1115 พร้อมใช้งาน ✓");

  writePCF(0x00);  // ดับ LED ทั้งหมด
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (now - lastUpdate >= 500) {
    lastUpdate = now;

    float voltage = readBatteryVoltage();

    if (voltage < 0) {
      Serial.println("Error: อ่านแรงดันไม่ได้");
      writePCF(0x00);
      noTone(BUZZER_PIN);
      return;
    }

    Serial.printf("แบต: %.3f V   ", voltage);

    if (voltage >= V_FULL)       Serial.print("[FULL] ");
    else if (voltage >= V_EMPTY) Serial.print("[OK] ");
    else if (voltage >= V_MIN_VALID) Serial.print("[LOW] ");
    else                         Serial.print("[NO BATTERY] ");

    updateLEDs(voltage);

    // เสียงเตือนเฉพาะช่วง 6V ถึง <11.11V
    if (voltage < V_EMPTY && voltage >= V_MIN_VALID) {
      Serial.println("!!! แบตต่ำ → เตือนเสียง !!!");
      buzzerAlert();
    } else {
      noTone(BUZZER_PIN);
      if (voltage < V_MIN_VALID) {
        Serial.println("ต่ำกว่า 6V → ไม่มีเสียง buzzer");
      }
    }
  }
}

// ── ฟังก์ชันต่าง ๆ ────────────────────────────────────────────────
float readBatteryVoltage() {
  const int samples = 10;
  long sum_raw = 0;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    int16_t raw = readADCConversion();
    if (raw != -1) {
      sum_raw += raw;
      valid++;
    }
    delay(3);
  }

  if (valid == 0) return -1.0f;

  float avg_raw = (float)sum_raw / valid;
  float v_a0 = avg_raw * (4.096f / 32768.0f);

  return v_a0 * CAL_SCALE + CAL_OFFSET;
}

int16_t readADCConversion() {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return -1;

  if (Wire.requestFrom(ADS_ADDR, (uint8_t)2) != 2) return -1;

  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw;
}

void updateLEDs(float v) {
  uint8_t pattern = 0x00;

  if (v >= V_FULL) {
    pattern = 0xFF;
  }
  else if (v >= V_EMPTY) {
    int steps = (int)round((v - V_EMPTY) / V_STEP);
    steps = constrain(steps, 0, 7);
    pattern = (1 << (steps + 1)) - 1;
  }
  else {
    // ต่ำกว่า V_EMPTY (ทั้ง <6V และ 6-11.11V) → ดวงล่างสุด + กระพริบ
    pattern = 0b00000001;
    unsigned long t = millis() % 400;
    if (t >= 200) pattern = 0x00;
  }

  writePCF(pattern);
}

void buzzerAlert() {
  static unsigned long lastBuzz = 0;
  unsigned long now = millis();

  if (now - lastBuzz < 5000) return;
  lastBuzz = now;

  Serial.println(">>> BUZZER ดังเตือน <<<");

  tone(BUZZER_PIN, 1400, 120); delay(180);
  tone(BUZZER_PIN, 1100, 120); delay(180);
  tone(BUZZER_PIN, 800, 180);  delay(250);

  noTone(BUZZER_PIN);
}

void writePCF(uint8_t value) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(value);
  Wire.endTransmission();
}