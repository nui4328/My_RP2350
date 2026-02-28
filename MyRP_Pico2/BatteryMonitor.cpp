// BatteryMonitor.cpp
#include "BatteryMonitor.h"

// ── กำหนดค่าคงที่จริง ๆ ที่นี่ ───────────────────────────────────
const uint8_t BatteryMonitor::ADS_ADDR     = 0x48;
const uint8_t BatteryMonitor::PCF_ADDR     = 0x20;
const uint8_t BatteryMonitor::BUZZER_PIN   = 9;

const float BatteryMonitor::CAL_SCALE      = 4.0296f;
const float BatteryMonitor::CAL_OFFSET     = 2.6699f;

const float BatteryMonitor::V_FULL         = 12.40f;
const float BatteryMonitor::V_EMPTY        = 11.11f;
const float BatteryMonitor::V_MIN_VALID    = 6.0f;
const float BatteryMonitor::V_STEP         = (V_FULL - V_EMPTY) / 7.0f;

const uint16_t BatteryMonitor::ADC_CONFIG  = 0x84C3;

// Constructor
BatteryMonitor::BatteryMonitor() 
  : _voltage(-1.0f), _lastUpdate(0), _lastBuzz(0) {
}

// เริ่มต้นใช้งาน
void BatteryMonitor::begin() {
  Wire.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  Serial.println("\n=== BatteryMonitor initialized ===");
  Serial.printf("V_FULL     = %.3f V\n", V_FULL);
  Serial.printf("V_EMPTY    = %.3f V\n", V_EMPTY);
  Serial.printf("V_MIN_VALID = %.1f V\n", V_MIN_VALID);

  // ตั้งค่า ADS1115 continuous mode
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(0x01);
  Wire.write(ADC_CONFIG >> 8);
  Wire.write(ADC_CONFIG & 0xFF);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: ADS1115 config failed!");
    while (true);
  }

  delay(50);
  Wire.beginTransmission(ADS_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: ADS1115 not found!");
    while (true);
  }

  writePCF(0x00);  // ดับ LED ทั้งหมด
}

// อัพเดทสถานะ (เรียกใน loop)
void BatteryMonitor::update() {
  unsigned long now = millis();
  if (now - _lastUpdate < 500) return;
  _lastUpdate = now;

  _voltage = readBatteryVoltage();

  if (_voltage < 0) {
    Serial.println("Error: อ่านแรงดันไม่ได้");
    writePCF(0x00);
    noTone(BUZZER_PIN);
    return;
  }

  Serial.print("แบต:  ");
  Serial.print(_voltage);
  Serial.println("V");

  if (_voltage >= V_FULL)       Serial.print("[FULL] ");
  else if (_voltage >= V_EMPTY) Serial.print("[OK] ");
  else if (_voltage >= V_MIN_VALID) Serial.print("[LOW] ");
  else                          Serial.print("[NO BATTERY] ");

  updateLEDs();

  // เสียงเตือนเฉพาะช่วง 6V ถึง <11.11V
  if (_voltage < V_EMPTY && _voltage >= V_MIN_VALID) {
    Serial.println("!!! แบตต่ำ → เตือนเสียง !!!");
    buzzerAlert();
  } else {
    noTone(BUZZER_PIN);
    if (_voltage < V_MIN_VALID) {
      Serial.println("ต่ำกว่า 6V → ไม่มีเสียง");
    }
  }
}

float BatteryMonitor::getVoltage() const {
  return _voltage;
}

bool BatteryMonitor::isLowBattery() const {
  return (_voltage < V_EMPTY && _voltage >= V_MIN_VALID);
}

bool BatteryMonitor::isNoBattery() const {
  return (_voltage < V_MIN_VALID);
}

bool BatteryMonitor::isNormal() const {
  return (_voltage >= V_EMPTY);
}

// ── ฟังก์ชันภายใน ─────────────────────────────────────────────────
float BatteryMonitor::readBatteryVoltage() {
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

int16_t BatteryMonitor::readADCConversion() {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return -1;

  if (Wire.requestFrom(ADS_ADDR, (uint8_t)2) != 2) return -1;

  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw;
}

void BatteryMonitor::updateLEDs() {
  uint8_t pattern = 0x00;

  if (_voltage >= V_FULL) {
    pattern = 0xFF;
  }
  else if (_voltage >= V_EMPTY) {
    int steps = (int)round((_voltage - V_EMPTY) / V_STEP);
    steps = constrain(steps, 0, 7);
    pattern = (1 << (steps + 1)) - 1;
  }
  else {
    pattern = 0b00000001;
    unsigned long t = millis() % 400;
    if (t >= 200) pattern = 0x00;
  }

  writePCF(pattern);
}

void BatteryMonitor::buzzerAlert() {
  unsigned long now = millis();
  if (now - _lastBuzz < 5000) return;
  _lastBuzz = now;

  Serial.println(">>> BUZZER ดังเตือน <<<");

  tone(BUZZER_PIN, 1400, 120); delay(180);
  tone(BUZZER_PIN, 1100, 120); delay(180);
  tone(BUZZER_PIN, 800, 180);  delay(250);

  noTone(BUZZER_PIN);
}

void BatteryMonitor::writePCF(uint8_t value) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(value);
  Wire.endTransmission();
}