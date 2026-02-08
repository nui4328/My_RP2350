#ifndef MY_TCS_H
#define MY_TCS_H

#include <Arduino.h>
#include <Wire.h>

#define TCS34725_ADDRESS          0x29
#define EXTERNAL_EEPROM_ADDR      0x50
#define COLOR_CAL_START_ADDR      0x0040

enum {
  TCS_IT_2_4MS  = 0xFF,
  TCS_IT_24MS   = 0xF6,
  TCS_IT_50MS   = 0xEB,
  TCS_IT_101MS  = 0xD5,   // แนะนำทั่วไป
  TCS_IT_154MS  = 0xC0,
  TCS_IT_180MS  = 0xB5,
  TCS_IT_614MS  = 0x00
};

enum {
  TCS_GAIN_1X  = 0x00,
  TCS_GAIN_4X  = 0x01,
  TCS_GAIN_16X = 0x02,
  TCS_GAIN_60X = 0x03
};

class my_TCS {
public:
    my_TCS(uint16_t eepromOffset = 0);

    bool begin(uint8_t addr = TCS34725_ADDRESS, TwoWire *theWire = &Wire);
    bool setConfig(uint8_t integrationTime = TCS_IT_101MS, uint8_t gain = TCS_GAIN_16X);

    bool readFast();

    void calibrateRed();
    void calibrateGreen();
    void calibrateBlue();
    void calibrateYellow();
    void calibrateWhite();
    void calibrateBlack();
    void calibrateDone();

    String getColor();

    void saveColorCalibration();
    void loadColorCalibration();

    void printRaw();
    void printCalibration();

    bool isCalibrated() const { return calibrated; }

    uint16_t c, r, g, b;

private:
    uint8_t _addr;
    TwoWire *_wire;
    uint16_t _eepromOffset;

    float ref_ratio_r[6];
    float ref_ratio_g[6];
    float ref_ratio_b[6];

    bool calibrated;
    bool cal_flags[6];

    void _calibrateSingle(uint8_t idx);

    void write8(uint8_t reg, uint8_t val);
    uint16_t read16(uint8_t reg);

    void writeEEPROM(uint16_t eeaddress, uint8_t data);
    uint8_t readEEPROM(uint16_t eeaddress);
    void writeEEPROM16(uint16_t eeaddress, uint16_t data);
    uint16_t readEEPROM16(uint16_t eeaddress);
};

#endif