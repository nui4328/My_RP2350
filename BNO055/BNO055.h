#ifndef BNO055_H
#define BNO055_H

#include <Arduino.h>
#include <Wire.h>

#define EEPROM_GYRO_BASE_ADDR   200
#define EEPROM_I2C_ADDR         0x50

class BNO055 {
public:
    bool begin(uint8_t addr = 0x29,
               TwoWire& wire = Wire,
               int sda = -1,
               int scl = -1,
               uint32_t clk = 400000);

    bool update();
    bool update_gyro();
    void resetAngles();
    float yaw();        
    float yawRaw();     
    float gyro(char xyz); 

    void calibrate(uint8_t buzzerPin = 0, bool forceNew = false);

private:
    TwoWire* _wire;
    uint8_t  _addr;

    float _yaw_raw = 0.0f;
    float _yaw = 0.0f;
    float _yaw_offset = 0.0f;

    float _dt = 0.0f;
    uint32_t _last_us = 0;
    float _alpha = 0.2f;

    float _gx = 0.0f;
    float _gy = 0.0f;
    float _gz = 0.0f;

    float _gyroOffsetX = 0.0f;
    float _gyroOffsetY = 0.0f;
    float _gyroOffsetZ = 0.0f;
    bool  _isGyroCalibrated = false;

    float _last_heading = 0.0f;     // สำคัญสำหรับ delta calculation

    // I2C helpers
    uint8_t read8(uint8_t reg);
    void    write8(uint8_t reg, uint8_t val);
    void    readGyro();
    void    applyGyroOffset();

    // EEPROM
    void    eepromWrite(uint16_t ee_addr, const uint8_t* data, uint8_t len);
    void    eepromRead(uint16_t ee_addr, uint8_t* data, uint8_t len);
    void    saveGyroCalibration();
    bool    loadGyroCalibration();
};

#endif