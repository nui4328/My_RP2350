#ifndef BNO055_CUSTOM_H
#define BNO055_CUSTOM_H

#include <Arduino.h>
#include <Wire.h>

#define EEPROM_I2C_ADDR       0x50
#define EEPROM_GYRO_BASE_ADDR 1100

class BNO055
{
public:
    bool begin(
        uint8_t addr,
        TwoWire& wire,
        int sda = -1,
        int scl = -1,
        uint32_t clk = 400000
    );

    bool update();
    bool update_gyro();

    float yaw() const;
    float yawRaw() const;
    float gyro(char xyz) const;
    float dt() const;

    void resetAngles();
    void setLPF(float alpha);

    void calibrate(
        uint8_t buzzerPin = 255,
        bool forceNew = false
    );

private:
    TwoWire* _wire = nullptr;
    uint8_t _addr = 0x28;

    float _yaw = 0.0f;
    float _yaw_raw = 0.0f;
    float _yaw_offset = 0.0f;

    float _last_heading = 0.0f;

    float _gx = 0.0f;
    float _gy = 0.0f;
    float _gz = 0.0f;

    float _gyroOffsetX = 0.0f;
    float _gyroOffsetY = 0.0f;
    float _gyroOffsetZ = 0.0f;

    float _alpha = 0.75f;

    float _dt = 0.0f;
    uint32_t _last_us = 0;

    bool _isGyroCalibrated = false;

    bool readHeading(float& heading);
    bool readGyro();

    void applyGyroOffset();

    bool eepromWrite(
        uint16_t ee_addr,
        const uint8_t* data,
        uint8_t len
    );

    bool eepromRead(
        uint16_t ee_addr,
        uint8_t* data,
        uint8_t len
    );

    void saveGyroCalibration();
    bool loadGyroCalibration();

    uint8_t read8(uint8_t reg);
    bool write8(uint8_t reg, uint8_t val);
};

#endif