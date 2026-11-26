#include "PiperPico2.h"

PiperPico2::PiperPico2(MotorDriver motorDriver)
  : _pwm(Adafruit_PWMServoDriver(0x40)),  // I2C address 0x40 (default ของ PCA9685)
    _display(128, 32, &Wire, -1, 100000, 100000)
{
    for (uint8_t i = 0; i < 4; ++i) {
        _motorDrivers[i] = motorDriver == MotorDriver::DRV8874
            ? MotorDriver::DRV8874 : MotorDriver::LEGACY;
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
        _sensorMin[i] = 0;
        _sensorMax[i] = 4095;
    }
}

bool PiperPico2::begin(bool initializeEncoders) {
    _boardConfigLocked = true;
    initPins(); // Disable drive PWM before touching I2C.
    _pwmReady = false;
    _displayReady = false;
    _motorOK = false;
    invalidateMotorCache();
    // Allow board supplies/peripherals to settle on a real power-on, not only
    // a warm MCU reset. Motor drive stays disabled throughout initialization.
    delay(250);
    const uint32_t started = millis();
    bool ready = false;
    do {
        _pwmReady = false;
        _displayReady = false;
        _motorOK = false;
        invalidateMotorCache();
        if (startI2C()) {
            const bool pwmOK = initPWM();
            const bool oledOK = initOLED();
            if (!pwmOK) _pwmReady = false;
            Wire.setClock(_i2cClock);
            ready = pwmOK && oledOK;
            if (ready) break;
        }
        delay(100);
    } while (millis() - started < 2000);
    if (!ready) return false;
    const bool encodersOK = !initializeEncoders || initEncoders();
    loadCalibration(); // Missing calibration does not prevent operation.
    updateBattery();
    return encodersOK;
}

bool PiperPico2::startI2C() {
    Wire.end(); // Release the controller before temporarily driving the pins.
    Wire.setSDA(4);
    Wire.setSCL(5);
    // Open-drain recovery: release HIGH, drive only LOW.
    pinMode(4, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
    // A peripheral may still hold a line during its own power-on reset.
    const uint32_t releasedAt = millis();
    while ((digitalRead(4) == LOW || digitalRead(5) == LOW) && millis() - releasedAt < 50) {
        delay(1);
    }
    for (uint8_t i = 0; i < 9 && digitalRead(4) == LOW; ++i) {
        digitalWrite(5, LOW);
        pinMode(5, OUTPUT);
        delayMicroseconds(5);
        pinMode(5, INPUT_PULLUP);
        delayMicroseconds(15);
    }
    // Generate STOP without ever driving a bus line HIGH.
    digitalWrite(5, LOW); pinMode(5, OUTPUT);
    digitalWrite(4, LOW); pinMode(4, OUTPUT);
    pinMode(5, INPUT_PULLUP); delayMicroseconds(15);
    pinMode(4, INPUT_PULLUP); delayMicroseconds(5);
    if (digitalRead(4) == LOW || digitalRead(5) == LOW) return false;
    Wire.begin();
    Wire.setClock(_i2cClock);
    Wire.setTimeout(25, true);
    return true;
}

bool PiperPico2::setI2CClock(uint32_t clockHz) {
    if (clockHz < 100000 || clockHz > 400000) return false;
    _i2cClock = clockHz;
    _display.setBusClock(clockHz);
    Wire.setClock(clockHz);
    return true;
}

void PiperPico2::initPins() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);

    pinMode(PWM_PIN_A, OUTPUT);
    pinMode(PWM_PIN_B, OUTPUT);
    pinMode(PWM_PIN_C, OUTPUT);
    pinMode(PWM_PIN_D, OUTPUT);

    analogWriteFreq(15000);
    analogWriteRange(PWM_MAX);
    analogReadResolution(12);
    analogWrite(PWM_PIN_A, 0);
    analogWrite(PWM_PIN_B, 0);
    analogWrite(PWM_PIN_C, 0);
    analogWrite(PWM_PIN_D, 0);
}

bool PiperPico2::initPWM() {
    _pwmReady = _pwm.begin();
    if (!_pwmReady) return false;
    Wire.setClock(_i2cClock);
    stopMotors();
    _pwm.setPWMFreq(SERVO_FREQUENCY_HZ);
    delay(3);                        // ให้ PCA9685 ปรับความถี่เสร็จก่อน
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t ch = SERVO_CHANNELS[i];
        _pwm.setPWM(ch, 0, 4096);     // Full OFF → ปิด servo ถูกต้อง
    }
    return _motorOK;
}

bool PiperPico2::setServoPulseRange(uint8_t logicalChannel, uint16_t minTicks, uint16_t maxTicks) {
    if (logicalChannel >= 6 || minTicks == 0 || minTicks >= maxTicks || maxTicks > 4095) {
        return false;
    }
    _servoMin[logicalChannel] = minTicks;
    _servoMax[logicalChannel] = maxTicks;
    _servoRangeSet[logicalChannel] = true;
    return true;
}

bool PiperPico2::setServoPulseRangeUs(uint8_t logicalChannel, uint16_t minUs, uint16_t maxUs) {
    if (logicalChannel >= 6 || minUs == 0 || minUs >= maxUs) return false;
    const uint32_t ticksPerSecond = uint32_t(SERVO_FREQUENCY_HZ) * 4096U;
    // Round the lower endpoint up and the upper endpoint down so quantization
    // never widens the requested nominal pulse window.
    const uint32_t minimum = (uint64_t(minUs) * ticksPerSecond + 999999U) / 1000000U;
    const uint32_t maximum = uint64_t(maxUs) * ticksPerSecond / 1000000U;
    if (minimum >= maximum || maximum > 4095) return false;
    return setServoPulseRange(logicalChannel, uint16_t(minimum), uint16_t(maximum));
}

void PiperPico2::setservo(uint8_t logicalChannel, int angle) {
    if (logicalChannel >= 6 || !_pwmReady) return;
    const uint16_t minimum = _servoRangeSet[logicalChannel] ? _servoMin[logicalChannel] : SERVO_MIN;
    const uint16_t maximum = _servoRangeSet[logicalChannel] ? _servoMax[logicalChannel] : SERVO_MAX;
    if (minimum == 0 || minimum >= maximum || maximum > 4095) return;
    uint8_t realChannel = SERVO_CHANNELS[logicalChannel];
    angle = constrain(angle, 0, 180);
    uint16_t pulse = map(angle, 0, 180, minimum, maximum);
    _pwm.setPWM(realChannel, 0, pulse);
}

bool PiperPico2::initOLED() {
    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() != 0) return false;
    _displayReady = _display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false);
    if (!_displayReady) return false;
    _display.clearDisplay();
    _display.setTextSize(2);
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(10, 0);
    //_display.println(F("PIPER"));
    //_display.println(F("ROBOTECH"));
    _display.display();
    Wire.setClock(_i2cClock);
    // begin() primarily reports buffer allocation, not successful I2C writes.
    Wire.beginTransmission(0x3C);
    _displayReady = Wire.endTransmission() == 0;
    return _displayReady;
}

void PiperPico2::setMotorPWMFrequency(uint32_t freqHz) {
    if (freqHz < 1000 || freqHz > 50000) {
        freqHz = 15000;
    }
    analogWriteFreq(freqHz);
}


void PiperPico2::playToneAsync(uint16_t freq, uint16_t duration_ms) {
    if (freq == 0 || duration_ms == 0) {
        noTone(BUZZER_PIN);
        return;
    }
    tone(BUZZER_PIN, freq, duration_ms);
}

void PiperPico2::playTone(uint16_t freq, uint16_t duration_ms) {
    if (freq == 0 || duration_ms == 0) return;
    playToneAsync(freq, duration_ms);
    delay(duration_ms); // Preserve the legacy synchronous API.
    noTone(BUZZER_PIN);
}

uint16_t PiperPico2::adcRead(uint8_t channel) {
    if (channel > 15) return 0;
    digitalWriteFast(MUX_S0, channel & 0x01);
    digitalWriteFast(MUX_S1, (channel >> 1) & 0x01);
    digitalWriteFast(MUX_S2, (channel >> 2) & 0x01);
    digitalWriteFast(MUX_S3, (channel >> 3) & 0x01);

    delayMicroseconds(2);

    uint32_t sum = 0;
    for (uint8_t i = 0; i < 2; i++) {
        sum += analogRead(MUX_Z);
    }
    return sum / 2;
}

uint16_t PiperPico2::adcMin(uint8_t sensor) {
    if (sensor >= NUM_SENSORS) return 0;
    if (_sensorMax[sensor] <= _sensorMin[sensor]) return 0;
    return _sensorMin[sensor];
}

uint16_t PiperPico2::adcMax(uint8_t sensor) {
    if (sensor >= NUM_SENSORS) return 4095;
    if (_sensorMax[sensor] <= _sensorMin[sensor]) return 4095;
    return _sensorMax[sensor];
}

uint16_t PiperPico2::adcMD(uint8_t sensor) {
    return (adcMin(sensor) + adcMax(sensor)) / 2;
}

uint16_t PiperPico2::readLine(uint8_t sensor) {
    if (sensor >= NUM_SENSORS) return 0;
    return adcRead(sensor);
}

void PiperPico2::readLines(uint16_t* rawValues) {
    if (!rawValues) return;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        rawValues[i] = adcRead(i);
    }
}

float PiperPico2::readNormalized(uint8_t sensor) {
    if (sensor >= NUM_SENSORS) return 0.0f;
    uint16_t val = adcRead(sensor);
    uint16_t minVal = adcMin(sensor);
    uint16_t maxVal = adcMax(sensor);
    if (maxVal <= minVal) return 0.0f;
    float norm = (float)(val - minVal) / (maxVal - minVal);
    return constrain(norm, 0.0f, 1.0f);
}

void PiperPico2::updateBattery() { _batteryVoltage = getVoltage(); }
float PiperPico2::getBatteryVoltage() { return _batteryVoltage; }
// Legacy helper: sample battery; presentation is explicitly requested separately.
void PiperPico2::bat_control() { updateBattery(); }

float PiperPico2::getVoltage() {
    uint16_t adc_value = adcRead(11);
    return (adc_value / 4095.0f) * 3.3f * VOLTAGE_DIVIDER_RATIO;
}

void PiperPico2::showVoltageUntilButton() {
    if (!_displayReady) return;
    stopMotors();
    while (true) {
        float voltage = getVoltage();

        _display.clearDisplay();
        _display.setTextSize(1);
        _display.setTextColor(SSD1306_WHITE);
        _display.setCursor(0, 0);
        _display.println("Battery Voltage");

        _display.setTextSize(2);
        _display.setCursor(8, 12);
        _display.print(voltage, 2);
        _display.print(" V");

        if (voltage < 10.0f) {
            _display.setTextSize(1);
            _display.setCursor(80, 25);
            _display.print("LOW BAT!");
        }

        if (digitalRead(BUTTON_PIN) == LOW) break;

        _display.display();
        Wire.setClock(_i2cClock);
        delay(100);
    }
    playTone(3000, 500);
    delay(30);
}

float PiperPico2::knopRead() {
    uint16_t knop_value = map(adcRead(10), 0, 2800, 0, 1203);
    return knop_value;
}

