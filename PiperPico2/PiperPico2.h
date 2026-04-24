#ifndef PIPER_PICO2_H
#define PIPER_PICO2_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "EncoderLibrarys.h"
#include "EncoderLibraryss.h"

#define NUM_SENSORS 10

#define PCA9685_MODE1      0x00
#define PCA9685_PRESCALE   0xFE

class MyPCA9685 {
public:
    MyPCA9685(uint8_t addr = 0x40);
    void begin();
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);
    void setPin(uint8_t channel, uint16_t val, bool invert = false);
    
    // เพิ่มฟังก์ชันนี้เพื่อ set ความถี่ PWM (สำคัญสำหรับเซอร์โว)
    void setPWMFreq(uint16_t freq);

private:
    uint8_t _addr;
    void write8(uint8_t reg, uint8_t val);
    uint8_t read8(uint8_t reg);
};

class PiperPico2 {
public:
    PiperPico2();

    bool begin();

    void motor(char id, int speed);
    void setServo(uint8_t logicalChannel, int angle);
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

    // เพิ่ม getter สำหรับเข้าถึง _pwm จากภายนอกถ้าต้องการทดสอบ (optional)
    MyPCA9685& getPWM() { return _pwm; }

private:
    MyPCA9685 _pwm;
    Adafruit_SSD1306 _display;

    EncoderLibrarys  _enc1;
    EncoderLibraryss _enc2;

    uint16_t _sensorMin[NUM_SENSORS];
    uint16_t _sensorMax[NUM_SENSORS];

    static constexpr uint8_t BUZZER_PIN  = 11;
    static constexpr uint8_t BUTTON_PIN  = 2;
    static constexpr uint8_t MUX_S0 = 13;
    static constexpr uint8_t MUX_S1 = 15;
    static constexpr uint8_t MUX_S2 = 12;
    static constexpr uint8_t MUX_S3 = 14;
    static constexpr uint8_t MUX_Z  = 28;
    

    // ค่า servo ที่ปรับแล้ว (150-600 เป็นค่าดีสำหรับส่วนใหญ่)
    static constexpr uint16_t SERVO_MIN = 135;
    static constexpr uint16_t SERVO_MAX = 580;

    static constexpr uint16_t PWM_MAX   = 65535;
    static constexpr float    SPEED_SCALE = PWM_MAX / 100.0f;
    static constexpr float    VOLTAGE_DIVIDER_RATIO = 7.96f;

    static constexpr uint8_t  EEPROM_I2C_ADDR  = 0x50;
    static constexpr uint16_t EEPROM_BASE_ADDR = 0x0000;

    static constexpr uint8_t  PWM_PIN_A = 8;
    static constexpr uint8_t  PWM_PIN_B = 9;
    static constexpr uint8_t  PWM_PIN_C = 7;
    static constexpr uint8_t  PWM_PIN_D = 6;

    // SERVO_CHANNELS[0] = 15 → เช็คว่าเซอร์โวต่อ channel 15 บน PCA9685 จริง
    static constexpr uint8_t SERVO_CHANNELS[5] = {15, 14, 13, 12, 11};

    uint8_t getMotorIn1(char id);
    uint8_t getMotorIn2(char id);

    void initPins();
    void initPWM();  // จะเรียก setPWMFreq(50) ตรงนี้
    void initOLED();
    void eepromWrite(uint16_t ee_addr, const uint8_t* data, uint8_t len);
    void eepromRead(uint16_t ee_addr, uint8_t* data, uint8_t len);
};

#endif