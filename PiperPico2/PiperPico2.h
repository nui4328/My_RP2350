#ifndef PIPER_PICO2_H
#define PIPER_PICO2_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>  // เปลี่ยนมาใช้ไลบรารี Adafruit

#include "EncoderLibrarys.h"
#include "EncoderLibraryss.h"

#define NUM_SENSORS 10

class PiperPico2 {
public:
    PiperPico2();

    bool begin();
    
    void updateBattery();
    float getBatteryVoltage();
    void bat_control();
    void motor(char id, int speed);
    void setservo(uint8_t logicalChannel, int angle);
    void playTone(uint16_t freq, uint16_t duration_ms);

    void readLines(uint16_t* rawValues);
    uint16_t readLine(uint8_t sensor);
    float readNormalized(uint8_t sensor);
    void calibrate();
    bool loadCalibration();
    bool saveCalibration();

    float getVoltage();
    void showVoltageUntilButton();

    void setMotorPWMFrequency(uint32_t freqHz);

    uint16_t adcRead(uint8_t channel);
    uint16_t adcMin(uint8_t sensor);
    uint16_t adcMax(uint8_t sensor);
    uint16_t adcMD(uint8_t sensor);

    long enc1Left()  { return _enc1.Poss_L(); }
    long enc1Right() { return _enc1.Poss_R(); }
    long enc2Left()  { return _enc2.Poss_L(); }
    long enc2Right() { return _enc2.Poss_R(); }
    void resetEncoders();
    float knopRead();

    // Getter สำหรับเข้าถึง pwm จากภายนอกถ้าต้องการทดสอบ (optional)
    Adafruit_PWMServoDriver& getPWM() { return _pwm; }
    uint16_t SERVO_MIN = 90;
    uint16_t SERVO_MAX = 760;
    unsigned long lastServoUpdate = 0;
    static constexpr uint8_t BUZZER_PIN  = 11;
    static constexpr uint8_t BUTTON_PIN  = 2;

private:
    Adafruit_PWMServoDriver _pwm;  // เปลี่ยนเป็น Adafruit library
    Adafruit_SSD1306 _display;

    EncoderLibrarys  _enc1;
    EncoderLibraryss _enc2;

    uint16_t _sensorMin[NUM_SENSORS];
    uint16_t _sensorMax[NUM_SENSORS];

    
    static constexpr uint8_t MUX_S0 = 13;
    static constexpr uint8_t MUX_S1 = 15;
    static constexpr uint8_t MUX_S2 = 12;
    static constexpr uint8_t MUX_S3 = 14;
    static constexpr uint8_t MUX_Z  = 28;

    // ค่า servo (ปรับตามที่คุณใช้ล่าสุด)
    

    uint16_t lastPulse[16] = {0};        // รองรับสูงสุด 16 ช่อง (PCA9685 มี 16)
    const unsigned long SERVO_UPDATE_INTERVAL = 50;  // ms  (20-30 แนะนำ)
    const uint16_t DEAD_BAND = 50;

    //----->>

    static constexpr uint16_t PWM_MAX   = 65535;
    static constexpr float    SPEED_SCALE = PWM_MAX / 100.0f;
    static constexpr float    VOLTAGE_DIVIDER_RATIO = 7.96f;

    static constexpr uint8_t  EEPROM_I2C_ADDR  = 0x50;
    static constexpr uint16_t EEPROM_BASE_ADDR = 0x0000;

    static constexpr uint8_t  PWM_PIN_A = 8;
    static constexpr uint8_t  PWM_PIN_B = 9;
    static constexpr uint8_t  PWM_PIN_C = 7;
    static constexpr uint8_t  PWM_PIN_D = 6;

    static constexpr uint8_t SERVO_CHANNELS[6] = {15, 14, 13, 12, 11,10};

    uint8_t getMotorIn1(char id);
    uint8_t getMotorIn2(char id);

    void initPins();
    void initPWM();
    void initOLED();
    void eepromWrite(uint16_t ee_addr, const uint8_t* data, uint8_t len);
    void eepromRead(uint16_t ee_addr, uint8_t* data, uint8_t len);
};

#endif