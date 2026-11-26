#ifndef PIPER_PICO2_H
#define PIPER_PICO2_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>  // เปลี่ยนมาใช้ไลบรารี Adafruit

#define NUM_SENSORS 10

class PiperPico2 {
public:
    enum class MotorDriver : uint8_t {
        LEGACY, // GPIO PWM plus two PCA9685 direction inputs.
        DRV8874 // PH/EN mode (PMODE wired to GND).
    };

    explicit PiperPico2(MotorDriver motorDriver = MotorDriver::LEGACY);
    ~PiperPico2();

    // Configure before the first begin(); false leaves configuration unchanged.
    bool setMotorDriver(char id, MotorDriver driver, bool reverseDirection = false);
    // Encoder index 0..3: enc1Left, enc1Right, enc2Left, enc2Right.
    // Before begin(), select normal/swapped order within the existing pin pair.
    bool setEncoderPins(uint8_t encoder, uint8_t pinA, uint8_t pinB);

    // Pass false when the application owns the encoder interrupts.
    bool begin(bool initializeEncoders = true);
    PiperPico2(const PiperPico2&) = delete;
    PiperPico2& operator=(const PiperPico2&) = delete;
    bool pwmReady() const { return _pwmReady; }
    bool displayReady() const { return _displayReady; }
    bool calibrationReady() const { return _calibrationReady; }
    bool motorOK() const { return _motorOK; }
    bool setI2CClock(uint32_t clockHz);
    void stopMotors();
    void invalidateMotorCache();
    // Call regularly for incremental calibration (millis-based).
    bool startCalibration(uint32_t durationMs = 10000);
    bool updateCalibration();
    void cancelCalibration();
    bool isCalibrating() const { return _calibrating; }
    bool calibrationSaved() const { return _calibrationSaved; }
    void playToneAsync(uint16_t freq, uint16_t duration_ms);
    
    void updateBattery();
    float getBatteryVoltage();
    void bat_control();
    void motor(char id, int speed);
    // Per logical channel 0..5. Configuration only; no movement or EEPROM write.
    // PWM counts: 1 <= minTicks < maxTicks <= 4095.
    bool setServoPulseRange(uint8_t logicalChannel, uint16_t minTicks, uint16_t maxTicks);
    // Microseconds at the library's nominal 60 Hz; rounded inside the limits.
    bool setServoPulseRangeUs(uint8_t logicalChannel, uint16_t minUs, uint16_t maxUs);
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

    long enc1Left()  { return _encoderCounts[0]; }
    long enc1Right() { return _encoderCounts[1]; }
    long enc2Left()  { return _encoderCounts[2]; }
    long enc2Right() { return _encoderCounts[3]; }
    void resetEncoders();
    float knopRead();

    // Getter สำหรับเข้าถึง pwm จากภายนอกถ้าต้องการทดสอบ (optional)
    Adafruit_PWMServoDriver& getPWM() { invalidateMotorCache(); return _pwm; }
    // Reuse the initialized OLED framebuffer; do not call begin() again.
    Adafruit_SSD1306& getDisplay() { return _display; }
    // Legacy defaults for channels without a per-channel override (PWM counts).
    uint16_t SERVO_MIN = 90;
    uint16_t SERVO_MAX = 760;
    unsigned long lastServoUpdate = 0;
    static constexpr uint8_t BUZZER_PIN  = 11;
    static constexpr uint8_t BUTTON_PIN  = 2;

private:
    class Display : public Adafruit_SSD1306 {
    public:
        using Adafruit_SSD1306::Adafruit_SSD1306;
        void setBusClock(uint32_t clockHz) {
            wireClk = clockHz;
            restoreClk = clockHz;
        }
    };
    Adafruit_PWMServoDriver _pwm;  // เปลี่ยนเป็น Adafruit library
    Display _display;

    // Only one board can own these physical encoder pins at a time.
    static PiperPico2* _encoderOwner;
    volatile int32_t _encoderCounts[4] = {0, 0, 0, 0};
    uint8_t _encoderPins[4][2] = {{20, 19}, {21, 22}, {16, 17}, {10, 18}};
    static void encoder0ISR();
    static void encoder1ISR();
    static void encoder2ISR();
    static void encoder3ISR();
    static void encoderEdge(uint8_t index);
    bool initEncoders();

    bool _pwmReady = false;
    bool _displayReady = false;
    bool _calibrationReady = false;
    bool _motorOK = false;
    bool _boardConfigLocked = false;
    MotorDriver _motorDrivers[4];
    bool _motorReversed[4] = {false, false, false, false};
    int8_t _motorDirections[4] = {2, 2, 2, 2};
    uint32_t _i2cClock = 100000;
    float _batteryVoltage = 0.0f;
    bool _calibrating = false;
    bool _calibrationSaved = false;
    uint32_t _calibrationStart = 0;
    uint32_t _calibrationDuration = 0;
    uint32_t _lastCalibrationSample = 0;
    uint16_t _pendingMin[NUM_SENSORS];
    uint16_t _pendingMax[NUM_SENSORS];

    uint16_t _sensorMin[NUM_SENSORS];
    uint16_t _sensorMax[NUM_SENSORS];

    
    static constexpr uint8_t MUX_S0 = 13;
    static constexpr uint8_t MUX_S1 = 15;
    static constexpr uint8_t MUX_S2 = 12;
    static constexpr uint8_t MUX_S3 = 14;
    static constexpr uint8_t MUX_Z  = 28;

    static constexpr uint16_t SERVO_FREQUENCY_HZ = 60;
    uint16_t _servoMin[6] = {};
    uint16_t _servoMax[6] = {};
    bool _servoRangeSet[6] = {};

    uint16_t lastPulse[16] = {0};        // รองรับสูงสุด 16 ช่อง (PCA9685 มี 16)
    const unsigned long SERVO_UPDATE_INTERVAL = 50;  // ms  (20-30 แนะนำ)
    const uint16_t DEAD_BAND = 50;

    //----->>

    static constexpr uint16_t PWM_MAX   = 65535;
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
    bool writeMotorDirection(uint8_t index, char id, int8_t direction);

    void initPins();
    bool startI2C();
    bool initPWM();
    bool initOLED();
    bool eepromWrite(uint16_t ee_addr, const uint8_t* data, uint8_t len);
    bool eepromRead(uint16_t ee_addr, uint8_t* data, uint8_t len);
};

#endif
