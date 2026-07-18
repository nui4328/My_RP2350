#include "BNO055.h"
#include <cmath>

// ======================================================
// BNO055 Registers
// ======================================================
#define BNO055_CHIP_ID         0x00
#define BNO055_PAGE_ID         0x07
#define BNO055_OPR_MODE        0x3D
#define BNO055_PWR_MODE        0x3E
#define BNO055_SYS_TRIGGER     0x3F

#define BNO055_EULER_H_LSB     0x1A
#define BNO055_GYR_DATA_X_LSB  0x14

#define BNO055_ID_VALUE        0xA0

#define MODE_CONFIG            0x00
#define MODE_NDOF              0x0C


// ======================================================
// ฟังก์ชันช่วยปรับมุมให้อยู่ในช่วง -180 ถึง +180
// ======================================================
static float wrapAngle180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}


// ======================================================
// BEGIN
// ======================================================
bool BNO055::begin(
    uint8_t addr,
    TwoWire& wire,
    int sda,
    int scl,
    uint32_t clk
)
{
    _addr = addr;
    _wire = &wire;

#if defined(ESP32)
    if (sda >= 0 && scl >= 0) {
        _wire->begin(sda, scl, clk);
    } else {
        _wire->begin();
        _wire->setClock(clk);
    }
#else
    _wire->begin();
    _wire->setClock(clk);
#endif

    delay(700);

    // ตรวจสอบ Chip ID หลายครั้ง
    bool found = false;

    for (uint8_t i = 0; i < 10; i++) {
        uint8_t chipID = read8(BNO055_CHIP_ID);

        if (chipID == BNO055_ID_VALUE) {
            found = true;
            break;
        }

        delay(100);
    }

    if (!found) {
        Serial.println(F("ไม่พบ BNO055 หรือ Chip ID ไม่ถูกต้อง"));
        return false;
    }

    // เข้า CONFIG mode ก่อนตั้งค่าต่าง ๆ
    write8(BNO055_OPR_MODE, MODE_CONFIG);
    delay(30);

    write8(BNO055_PAGE_ID, 0x00);
    delay(10);

    // Normal power mode
    write8(BNO055_PWR_MODE, 0x00);
    delay(10);

    // ใช้ external crystal ถ้ามีบนโมดูล
    write8(BNO055_SYS_TRIGGER, 0x80);
    delay(10);

    // เข้า NDOF mode
    write8(BNO055_OPR_MODE, MODE_NDOF);
    delay(50);

    _last_us = micros();

    resetAngles();

    Serial.println(F("BNO055 initialized successfully"));

    return true;
}


// ======================================================
// อ่านค่า Heading แบบ Absolute 0–360 องศา
// ======================================================
bool BNO055::readHeading(float& heading)
{
    if (_wire == nullptr) {
        return false;
    }

    _wire->beginTransmission(_addr);
    _wire->write(BNO055_EULER_H_LSB);

    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    uint8_t count = _wire->requestFrom(_addr, (uint8_t)2);

    if (count != 2 || _wire->available() < 2) {
        return false;
    }

    uint8_t lsb = _wire->read();
    uint8_t msb = _wire->read();

    int16_t raw = (int16_t)(((uint16_t)msb << 8) | lsb);

    heading = raw / 16.0f;

    return true;
}


// ======================================================
// UPDATE
//
// ใช้ Euler heading จาก BNO055 แล้วแปลงเป็นมุมสะสมต่อเนื่อง
// ตัวอย่าง:
// 350, 355, 359, 0, 5
// จะได้:
// 0, 5, 9, 10, 15
// ======================================================
bool BNO055::update()
{
    uint32_t now = micros();

    if (_last_us == 0) {
        _last_us = now;
    }

    _dt = (now - _last_us) * 1.0e-6f;
    _last_us = now;

    // ป้องกัน dt ผิดปกติ
    if (_dt <= 0.0f || _dt > 0.5f) {
        _dt = 0.01f;
    }

    float headingAbsolute;

    if (!readHeading(headingAbsolute)) {
        return false;
    }

    /*
     * หาความต่างระหว่าง heading ปัจจุบันกับครั้งก่อน
     * แล้วปรับให้เป็นช่วง -180 ถึง +180
     */
    float delta = wrapAngle180(
        headingAbsolute - _last_heading
    );

    _last_heading = headingAbsolute;

    // สะสมเป็นมุมต่อเนื่อง
    _yaw_raw += delta;

    /*
     * Low-pass filter
     *
     * alpha สูง = นิ่ง แต่ตอบสนองช้า
     * alpha ต่ำ = ตอบสนองเร็ว
     *
     * เช่น 0.70–0.90
     */
    _yaw = (_alpha * _yaw) +
           ((1.0f - _alpha) * _yaw_raw);

    if (!readGyro()) {
        return false;
    }

    applyGyroOffset();

    return true;
}


// ======================================================
// UPDATE_GYRO
//
// ใช้โครงสร้างเดียวกับ update()
// ป้องกันมุมข้าม 0/360 แล้วกระโดด
// ======================================================
bool BNO055::update_gyro()
{
    return update();
}


// ======================================================
// API
// ======================================================
float BNO055::yaw() const
{
    return _yaw;
}


float BNO055::yawRaw() const
{
    return _yaw_raw;
}


float BNO055::gyro(char xyz) const
{
    switch (xyz) {
        case 'x':
        case 'X':
            return _gx;

        case 'y':
        case 'Y':
            return _gy;

        case 'z':
        case 'Z':
            return _gz;

        default:
            return 0.0f;
    }
}


float BNO055::dt() const
{
    return _dt;
}


void BNO055::setLPF(float alpha)
{
    // จำกัดค่าไม่ให้ผิดช่วง
    _alpha = constrain(alpha, 0.0f, 0.99f);
}


// ======================================================
// RESET ANGLES
// ======================================================
void BNO055::resetAngles()
{
    float currentAbsolute = 0.0f;

    if (!readHeading(currentAbsolute)) {
        Serial.println(F("Reset yaw ไม่สำเร็จ: อ่าน Heading ไม่ได้"));

        _yaw = 0.0f;
        _yaw_raw = 0.0f;
        _last_heading = 0.0f;
        _yaw_offset = 0.0f;
        _last_us = micros();

        return;
    }

    /*
     * _last_heading ต้องเป็นค่า Absolute จริง
     * เพราะ update() จะนำ Absolute ใหม่มาลบกับค่านี้
     */
    _last_heading = currentAbsolute;

    // เก็บไว้สำหรับแสดงผลเท่านั้น
    _yaw_offset = currentAbsolute;

    _yaw = 0.0f;
    _yaw_raw = 0.0f;

    _last_us = micros();
    _dt = 0.0f;

    Serial.printf(
        "RESET YAW COMPLETE | Heading = %.2f deg | yaw = 0.00\n",
        currentAbsolute
    );
}


// ======================================================
// CALIBRATE GYRO
// ======================================================
void BNO055::calibrate(uint8_t buzzerPin, bool forceNew)
{
    bool useSound = (buzzerPin != 255);

    if (useSound) {
        pinMode(buzzerPin, OUTPUT);
        digitalWrite(buzzerPin, LOW);

        tone(buzzerPin, 2000, 150);
        delay(200);
    }

    // โหลดค่าที่เคยบันทึกไว้
    if (!forceNew && loadGyroCalibration()) {
        Serial.println(F("โหลด Gyro calibration จาก EEPROM สำเร็จ"));

        Serial.printf(
            "Gyro offsets: X=%.4f Y=%.4f Z=%.4f deg/s\n",
            _gyroOffsetX,
            _gyroOffsetY,
            _gyroOffsetZ
        );

        if (useSound) {
            tone(buzzerPin, 1800, 100);
            delay(150);

            tone(buzzerPin, 2200, 150);
        }

        resetAngles();
        return;
    }

    Serial.println(F("เริ่มเก็บข้อมูล Gyro"));
    Serial.println(F("กรุณาวางหุ่นยนต์ให้นิ่ง"));

    /*
     * สำคัญ:
     * ปิดการใช้ Offset เดิมขณะเก็บค่าใหม่
     */
    bool previousCalibrationState = _isGyroCalibrated;
    _isGyroCalibrated = false;

    const uint16_t samples = 300;

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;

    uint16_t validSamples = 0;

    unsigned long startTime = millis();
    unsigned long lastBeep = startTime;

    while (
        validSamples < samples &&
        millis() - startTime < 6000UL
    ) {
        if (readGyro()) {
            sumX += _gx;
            sumY += _gy;
            sumZ += _gz;

            validSamples++;
        }

        if (
            useSound &&
            millis() - lastBeep >= 300
        ) {
            tone(buzzerPin, 1800, 60);
            lastBeep = millis();
        }

        delay(10);
    }

    if (useSound) {
        noTone(buzzerPin);
    }

    if (validSamples >= 200) {
        _gyroOffsetX = sumX / validSamples;
        _gyroOffsetY = sumY / validSamples;
        _gyroOffsetZ = sumZ / validSamples;

        _isGyroCalibrated = true;

        Serial.println(F("Calibration สำเร็จ"));

        Serial.printf(
            "Gyro Offset X = %.5f deg/s\n",
            _gyroOffsetX
        );

        Serial.printf(
            "Gyro Offset Y = %.5f deg/s\n",
            _gyroOffsetY
        );

        Serial.printf(
            "Gyro Offset Z = %.5f deg/s\n",
            _gyroOffsetZ
        );

        saveGyroCalibration();
        resetAngles();

        if (useSound) {
            tone(buzzerPin, 1600, 100);
            delay(130);

            tone(buzzerPin, 2100, 100);
            delay(130);

            tone(buzzerPin, 2500, 200);
        }
    } else {
        _isGyroCalibrated = previousCalibrationState;

        Serial.printf(
            "Calibration ไม่สำเร็จ อ่านได้เพียง %u samples\n",
            validSamples
        );

        if (useSound) {
            tone(buzzerPin, 400, 600);
        }
    }
}


// ======================================================
// APPLY GYRO OFFSET
// ======================================================
void BNO055::applyGyroOffset()
{
    if (!_isGyroCalibrated) {
        return;
    }

    _gx -= _gyroOffsetX;
    _gy -= _gyroOffsetY;
    _gz -= _gyroOffsetZ;

    // Deadband ลด Noise ขณะหยุดนิ่ง
    if (fabs(_gx) < 0.03f) _gx = 0.0f;
    if (fabs(_gy) < 0.03f) _gy = 0.0f;
    if (fabs(_gz) < 0.03f) _gz = 0.0f;
}


// ======================================================
// READ RAW GYRO
// ======================================================
bool BNO055::readGyro()
{
    if (_wire == nullptr) {
        return false;
    }

    _wire->beginTransmission(_addr);
    _wire->write(BNO055_GYR_DATA_X_LSB);

    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    uint8_t count = _wire->requestFrom(
        _addr,
        (uint8_t)6
    );

    if (count != 6 || _wire->available() < 6) {
        return false;
    }

    int16_t x =
        (int16_t)(
            _wire->read() |
            ((uint16_t)_wire->read() << 8)
        );

    int16_t y =
        (int16_t)(
            _wire->read() |
            ((uint16_t)_wire->read() << 8)
        );

    int16_t z =
        (int16_t)(
            _wire->read() |
            ((uint16_t)_wire->read() << 8)
        );

    /*
     * BNO055 Gyro หน่วย:
     * 16 LSB = 1 deg/s
     */
    _gx = x / 16.0f;
    _gy = y / 16.0f;
    _gz = z / 16.0f;

    return true;
}


// ======================================================
// EEPROM WRITE
// ======================================================
bool BNO055::eepromWrite(
    uint16_t ee_addr,
    const uint8_t* data,
    uint8_t len
)
{
    if (_wire == nullptr || data == nullptr || len == 0) {
        return false;
    }

    _wire->beginTransmission(EEPROM_I2C_ADDR);

    _wire->write(
        (uint8_t)((ee_addr >> 8) & 0xFF)
    );

    _wire->write(
        (uint8_t)(ee_addr & 0xFF)
    );

    for (uint8_t i = 0; i < len; i++) {
        _wire->write(data[i]);
    }

    uint8_t result = _wire->endTransmission();

    delay(8);

    return result == 0;
}


// ======================================================
// EEPROM READ
// ======================================================
bool BNO055::eepromRead(
    uint16_t ee_addr,
    uint8_t* data,
    uint8_t len
)
{
    if (_wire == nullptr || data == nullptr || len == 0) {
        return false;
    }

    _wire->beginTransmission(EEPROM_I2C_ADDR);

    _wire->write(
        (uint8_t)((ee_addr >> 8) & 0xFF)
    );

    _wire->write(
        (uint8_t)(ee_addr & 0xFF)
    );

    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    uint8_t received = _wire->requestFrom(
        EEPROM_I2C_ADDR,
        len
    );

    if (received != len) {
        return false;
    }

    uint8_t index = 0;

    while (_wire->available() && index < len) {
        data[index++] = _wire->read();
    }

    return index == len;
}


// ======================================================
// SAVE GYRO CALIBRATION
// ======================================================
void BNO055::saveGyroCalibration()
{
    uint8_t buffer[13];

    memcpy(
        &buffer[0],
        &_gyroOffsetX,
        sizeof(float)
    );

    memcpy(
        &buffer[4],
        &_gyroOffsetY,
        sizeof(float)
    );

    memcpy(
        &buffer[8],
        &_gyroOffsetZ,
        sizeof(float)
    );

    buffer[12] = 0xAA;

    if (
        eepromWrite(
            EEPROM_GYRO_BASE_ADDR,
            buffer,
            sizeof(buffer)
        )
    ) {
        Serial.println(
            F("บันทึก Gyro Offset ลง EEPROM สำเร็จ")
        );
    } else {
        Serial.println(
            F("บันทึก Gyro Offset ลง EEPROM ไม่สำเร็จ")
        );
    }
}


// ======================================================
// LOAD GYRO CALIBRATION
// ======================================================
bool BNO055::loadGyroCalibration()
{
    uint8_t buffer[13];

    memset(buffer, 0, sizeof(buffer));

    if (
        !eepromRead(
            EEPROM_GYRO_BASE_ADDR,
            buffer,
            sizeof(buffer)
        )
    ) {
        return false;
    }

    if (buffer[12] != 0xAA) {
        return false;
    }

    memcpy(
        &_gyroOffsetX,
        &buffer[0],
        sizeof(float)
    );

    memcpy(
        &_gyroOffsetY,
        &buffer[4],
        sizeof(float)
    );

    memcpy(
        &_gyroOffsetZ,
        &buffer[8],
        sizeof(float)
    );

    // ตรวจสอบค่าผิดปกติจาก EEPROM
    if (
        !isfinite(_gyroOffsetX) ||
        !isfinite(_gyroOffsetY) ||
        !isfinite(_gyroOffsetZ)
    ) {
        return false;
    }

    if (
        fabs(_gyroOffsetX) > 50.0f ||
        fabs(_gyroOffsetY) > 50.0f ||
        fabs(_gyroOffsetZ) > 50.0f
    ) {
        return false;
    }

    _isGyroCalibrated = true;

    return true;
}


// ======================================================
// I2C READ 8-BIT
// ======================================================
uint8_t BNO055::read8(uint8_t reg)
{
    if (_wire == nullptr) {
        return 0xFF;
    }

    _wire->beginTransmission(_addr);
    _wire->write(reg);

    if (_wire->endTransmission(false) != 0) {
        return 0xFF;
    }

    uint8_t received = _wire->requestFrom(
        _addr,
        (uint8_t)1
    );

    if (received != 1 || !_wire->available()) {
        return 0xFF;
    }

    return (uint8_t)_wire->read();
}


// ======================================================
// I2C WRITE 8-BIT
// ======================================================
bool BNO055::write8(uint8_t reg, uint8_t val)
{
    if (_wire == nullptr) {
        return false;
    }

    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);

    return _wire->endTransmission() == 0;
}