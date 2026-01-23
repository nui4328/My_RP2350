#ifndef BORD_ARRAY_H
#define BORD_ARRAY_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// ปรับ ratio ตามการวัดจริง: แบต 7.82V → ADC ≈1158 → ratio ≈8.38
#define VOLTAGE_DIVIDER_RATIO 8.38f  

class ArrayPico2 {
public:
    // Pin Definitions
    static constexpr uint8_t MUX_S0 = 13;
    static constexpr uint8_t MUX_S1 = 15;
    static constexpr uint8_t MUX_S2 = 12;
    static constexpr uint8_t MUX_S3 = 14;
    static constexpr uint8_t MUX_Z  = 28;

    static constexpr uint8_t PWMA  = 8;
    static constexpr uint8_t AIN1  = 10;
    static constexpr uint8_t AIN2  = 9;
    static constexpr uint8_t PWMB  = 7;
    static constexpr uint8_t BIN1  = 3;
    static constexpr uint8_t BIN2  = 6;

    static constexpr uint8_t BUZZER_PIN = 11;
    static constexpr uint8_t BUTTON_PIN = 2;

    // Constants
    static constexpr uint8_t EEPROM_ADDRESS = 0x50;
    static constexpr uint8_t SCREEN_ADDRESS = 0x3C;

    static constexpr uint8_t NUM_SENSORS     = 10;
    static constexpr uint8_t SAMPLES_PER_READ = 4;

    // Public Methods
    void begin();
    void run();
    void update();

    // Motor Control
    void Motor(int leftSpeed, int rightSpeed);
    void forward(int speed = 100);
    void backward(int speed = 100);
    void turnLeft(int speed = 80);
    void turnRight(int speed = 80);
    void stop();

    // PWM Frequency
    void setMotorPWMFrequency(uint32_t freqHz);
    uint32_t getMotorPWMFrequency() const;

    // Servo Control
    void ServoAttach(uint8_t pin);
    void ServoWrite(uint8_t pin, int degree);
    void ServoDetach(uint8_t pin);

    // Sensor Reading
    uint16_t adcRead(uint8_t channel);
    uint16_t adcMD(uint8_t channel);
    uint16_t adcMax(uint8_t channel);
    uint16_t adcMin(uint8_t channel);

    // Display & Utility
    void displayVoltage();

private:
    Adafruit_SSD1306 display{128, 32, &Wire, -1};

    uint16_t sensorValues[NUM_SENSORS]{};
    uint16_t sensorMin[NUM_SENSORS]{};
    uint16_t sensorMax[NUM_SENSORS]{};

    uint32_t motor_pwm_freq = 16000;

    // Private Methods
    void playStartupSound();
    void loadCalibrationFromEEPROM();
    void saveCalibrationToEEPROM();
    void calibrateSensors();
    void viewSensorValues();
    void showWelcomeScreen();
    void showMainMenu();
    void waitForButtonAction();
    void enterSettingsMode();

    void writeWordToEEPROM(uint16_t eeaddr, uint16_t data);
    uint16_t readWordFromEEPROM(uint16_t eeaddr);

    Servo servo16;
    Servo servo17;
    Servo servo18;
};

#endif  // BORD_ARRAY_H