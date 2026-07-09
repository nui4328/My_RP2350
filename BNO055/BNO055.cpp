#include "BNO055.h"
#include <cmath>

// -------------------- Registers --------------------
#define BNO055_CHIP_ID        0x00
#define BNO055_OPR_MODE       0x3D
#define BNO055_EULER_H_LSB    0x1A
#define BNO055_GYR_DATA_X_LSB 0x14

// -------------------- begin --------------------
bool BNO055::begin(uint8_t addr, TwoWire& wire, int sda, int scl, uint32_t clk) {
    _addr = addr;
    _wire = &wire;

#if defined(ESP32)
    if (sda >= 0 && scl >= 0)
        _wire->begin(sda, scl, clk);
    else
        _wire->begin();
#else
    _wire->begin();
#endif

    delay(200);

    if (read8(BNO055_CHIP_ID) != 0xA0)
        return false;

    write8(BNO055_OPR_MODE, 0x0C); // NDOF mode
    delay(20);

    _last_us = micros();
    resetAngles();

    Serial.println(F("BNO055 initialized successfully"));
    return true;
}

// -------------------- update --------------------
bool BNO055::update() {
    uint32_t now = micros();
    _dt = (now - _last_us) * 1e-6f;
    _last_us = now;

    uint8_t lsb = read8(BNO055_EULER_H_LSB);
    uint8_t msb = read8(BNO055_EULER_H_LSB + 1);
    int16_t raw = (int16_t)((msb << 8) | lsb);

    float y = raw / 16.0f - _yaw_offset;

    static float last = 0;
    float diff = y - last;
    if (diff > 180)  y -= 360;
    if (diff < -180) y += 360;
    last = y;

    _yaw_raw = y;
    _yaw += _alpha * (_yaw_raw - _yaw);

    readGyro();
    applyGyroOffset();

    return true;
}

// -------------------- update_gyro --------------------
bool BNO055::update_gyro() {
    uint32_t now = micros();
    _dt = (now - _last_us) * 1e-6f;
    _last_us = now;

    uint8_t lsb = read8(BNO055_EULER_H_LSB);
    uint8_t msb = read8(BNO055_EULER_H_LSB + 1);
    int16_t raw = (int16_t)((msb << 8) | lsb);

    float heading = (raw / 16.0f) - _yaw_offset;

    float delta = heading - _last_heading;

    _last_heading = heading;

    _yaw_raw += delta;
    _yaw += _alpha * (_yaw_raw - _yaw);

    

    readGyro();
    applyGyroOffset();

    return true;
}

// -------------------- API --------------------
float BNO055::yaw() {
    return _yaw;
}

float BNO055::yawRaw() {
    return _yaw_raw;
}

float BNO055::gyro(char xyz) {
    if (xyz == 'x' || xyz == 'X') return _gx;
    if (xyz == 'y' || xyz == 'Y') return _gy;
    if (xyz == 'z' || xyz == 'Z') return _gz;     // ต้องเป็น rate
    return 0;
}

void BNO055::resetAngles() {
    uint8_t lsb = read8(BNO055_EULER_H_LSB);
    uint8_t msb = read8(BNO055_EULER_H_LSB + 1);
    int16_t raw = (int16_t)((msb << 8) | lsb);
    float current_absolute = raw / 16.0f;

    _yaw_offset = current_absolute;

    _yaw = 0.0f;
    _yaw_raw = 0.0f;
    _last_heading = current_absolute;

    _last_us = micros();
    _dt = 0.0f;

    Serial.printf("🔄 RESET YAW COMPLETE | Offset = %.2f° | yaw = 0.00\n", _yaw_offset);
}

// ==================== Calibrate ====================
void BNO055::calibrate(uint8_t buzzerPin, bool forceNew) {
    bool useSound = (buzzerPin > 0);
    if (useSound) {
        pinMode(buzzerPin, OUTPUT);
        digitalWrite(buzzerPin, LOW);
        tone(buzzerPin, 2000, 150);
        delay(100);
    }

    if (!forceNew && loadGyroCalibration()) {
        if (useSound) {
            tone(buzzerPin, 1800, 100); delay(150);
            tone(buzzerPin, 2200, 150);
        }
        resetAngles();
        return;
    }

    Serial.println(F("เริ่มเก็บข้อมูล Gyro..."));

    const int samples = 250;
    float sumX = 0, sumY = 0, sumZ = 0;
    int count = 0;

    unsigned long startTime = millis();
    unsigned long lastBeep = startTime;

    while (count < samples && (millis() - startTime < 4500UL)) {
        update();
        sumX += _gx; sumY += _gy; sumZ += _gz;
        count++;

        if (useSound && (millis() - lastBeep >= 250)) {
            tone(buzzerPin, 1800, 80);
            lastBeep = millis();
        }
        delay(10);
    }

    if (useSound) noTone(buzzerPin);

    if (count > 120) {
        _gyroOffsetX = sumX / count;
        _gyroOffsetY = sumY / count;
        _gyroOffsetZ = sumZ / count;
        _isGyroCalibrated = true;

        Serial.printf("✅ Calibration สำเร็จ! Gyro Offset Z = %.4f deg/s\n", _gyroOffsetZ);
        saveGyroCalibration();
        resetAngles();
    } else {
        Serial.println(F("⚠️ Calibration ไม่สำเร็จ"));
        if (useSound) tone(buzzerPin, 400, 600);
    }
}

// ==================== Helper ====================
void BNO055::applyGyroOffset() {
    if (_isGyroCalibrated) {
        _gx -= _gyroOffsetX;
        _gy -= _gyroOffsetY;
        _gz -= _gyroOffsetZ;
    }
}

// ==================== EEPROM ====================
void BNO055::eepromWrite(uint16_t ee_addr, const uint8_t* data, uint8_t len) {
    _wire->beginTransmission(EEPROM_I2C_ADDR);
    _wire->write((ee_addr >> 8) & 0xFF);
    _wire->write(ee_addr & 0xFF);
    for (uint8_t i = 0; i < len; i++) _wire->write(data[i]);
    _wire->endTransmission();
    delay(5);
}

void BNO055::eepromRead(uint16_t ee_addr, uint8_t* data, uint8_t len) {
    _wire->beginTransmission(EEPROM_I2C_ADDR);
    _wire->write((ee_addr >> 8) & 0xFF);
    _wire->write(ee_addr & 0xFF);
    _wire->endTransmission();

    _wire->requestFrom(EEPROM_I2C_ADDR, len);
    for (uint8_t i = 0; i < len && _wire->available(); i++) {
        data[i] = _wire->read();
    }
}

void BNO055::saveGyroCalibration() {
    uint8_t buffer[13];
    uint16_t addr = EEPROM_GYRO_BASE_ADDR;

    memcpy(&buffer[0], &_gyroOffsetX, sizeof(float));
    memcpy(&buffer[4], &_gyroOffsetY, sizeof(float));
    memcpy(&buffer[8], &_gyroOffsetZ, sizeof(float));
    buffer[12] = 0xAA;

    eepromWrite(addr, buffer, 13);
    Serial.println(F("💾 บันทึก Gyro Offset ลง EEPROM เรียบร้อย"));
}

bool BNO055::loadGyroCalibration() {
    uint8_t buffer[13];
    uint16_t addr = EEPROM_GYRO_BASE_ADDR;

    eepromRead(addr, buffer, 13);
    if (buffer[12] != 0xAA) return false;

    memcpy(&_gyroOffsetX, &buffer[0], sizeof(float));
    memcpy(&_gyroOffsetY, &buffer[4], sizeof(float));
    memcpy(&_gyroOffsetZ, &buffer[8], sizeof(float));

    _isGyroCalibrated = true;
    return true;
}

void BNO055::readGyro() {
    _wire->beginTransmission(_addr);
    _wire->write(BNO055_GYR_DATA_X_LSB);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)6);

    int16_t x = _wire->read() | (_wire->read() << 8);
    int16_t y = _wire->read() | (_wire->read() << 8);
    int16_t z = _wire->read() | (_wire->read() << 8);

    _gx = x / 16.0f;
    _gy = y / 16.0f;
    _gz = z / 16.0f;
}

// ==================== I2C ====================
uint8_t BNO055::read8(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)1);

    return _wire->read();        // ← แก้ไขตรงนี้ (เพิ่มวงเล็บ)
}

void BNO055::write8(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}