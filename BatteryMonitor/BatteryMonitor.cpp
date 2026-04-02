// BatteryMonitor.cpp
#include "BatteryMonitor.h"

// ── ค่าคงที่ ───────────────────────────────────
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
    : _voltage(11.8f),
      _rawVoltage(11.8f),
      _lastUpdate(0),
      _lastBuzz(0),
      _debug(false) {
}

// เริ่มต้นใช้งาน
void BatteryMonitor::begin() {
    Wire.begin();

    pinMode(BUZZER_PIN, OUTPUT);
    noTone(BUZZER_PIN);
    writePCF(0x00);

    // ตั้งค่า ADS1115
    Wire.beginTransmission(ADS_ADDR);
    Wire.write(0x01);
    Wire.write(ADC_CONFIG >> 8);
    Wire.write(ADC_CONFIG & 0xFF);
    Wire.endTransmission();

    Serial.println("=== BatteryMonitor initialized (Non-blocking mode) ===");
}

void BatteryMonitor::setDebug(bool enable) {
    _debug = enable;
}

// อัพเดทสถานะแบบ Non-Blocking
void BatteryMonitor::update() {
    unsigned long now = millis();

    if (now - _lastUpdate < 100) return;   // อ่านจริงทุก 150ms
    _lastUpdate = now;

    // อ่าน ADC แบบเร็ว
    int16_t raw = readADCConversion();
    if (raw != -1) {
        float v_a0 = raw * (4.096f / 32768.0f);
        float newV = v_a0 * CAL_SCALE + CAL_OFFSET;

        _rawVoltage = newV;
        _voltage = 0.87f * _voltage + 0.13f * newV;   // Filter นุ่มนวล
    }

    if (_debug) {
        Serial.print("BAT: ");
        Serial.print(_voltage, 3);
        Serial.println(" V");
    }

    updateLEDs();

    if (_voltage < V_EMPTY && _voltage >= V_MIN_VALID) {
        buzzerAlertNonBlocking();
    } else {
        noTone(BUZZER_PIN);
    }
}

float BatteryMonitor::getVoltage() const {
    return _voltage;
}

float BatteryMonitor::getRawVoltage() const {
    return _rawVoltage;
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

int16_t BatteryMonitor::readADCConversion() {
    Wire.beginTransmission(ADS_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return -1;

    if (Wire.requestFrom(ADS_ADDR, (uint8_t)2) != 2) return -1;

    return (Wire.read() << 8) | Wire.read();
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
        if ((millis() % 400) >= 200) pattern = 0x00;
    }

    writePCF(pattern);
}

void BatteryMonitor::buzzerAlertNonBlocking() {
    unsigned long now = millis();
    if (now - _lastBuzz < 5000) return;
    _lastBuzz = now;

    tone(BUZZER_PIN, 1400, 120);
}

void BatteryMonitor::writePCF(uint8_t value) {
    Wire.beginTransmission(PCF_ADDR);
    Wire.write(value);
    Wire.endTransmission();
}

// ================================================
// ฟังก์ชันชื่อเดิม updateBattery() สำหรับ compatibility
// ================================================

