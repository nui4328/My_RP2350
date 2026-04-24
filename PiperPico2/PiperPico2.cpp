#include "PiperPico2.h"

#define PCA9685_MODE1      0x00
#define PCA9685_PRESCALE   0xFE

MyPCA9685::MyPCA9685(uint8_t addr) : _addr(addr) {}

void MyPCA9685::write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t MyPCA9685::read8(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(_addr, (uint8_t)1);
    if (Wire.available()) return Wire.read();
    return 0;
}

void MyPCA9685::begin() {
    write8(PCA9685_MODE1, 0x00);
    delay(10);

    // prescale เดิมถูกแทนที่ด้วย setPWMFreq(50) ใน initPWM แล้ว
    // ถ้าต้องการค่า default สามารถคงไว้ แต่แนะนำใช้ setPWMFreq
}

void MyPCA9685::setPWMFreq(uint16_t freq) {
    // คำนวณ prescale สำหรับ internal oscillator 25 MHz
    // freq ควรอยู่ระหว่าง 40-1000 Hz (สำหรับ servo ใช้ 50-60 Hz)
    if (freq < 40) freq = 40;
    if (freq > 1000) freq = 1000;

    float prescaleval = 25000000.0f;  // 25 MHz
    prescaleval /= 4096.0f;
    prescaleval /= (float)freq;
    prescaleval -= 1.0f;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);  // round to nearest

    uint8_t oldmode = read8(PCA9685_MODE1);
    write8(PCA9685_MODE1, (oldmode & 0x7F) | 0x10);  // sleep mode
    delay(5);
    write8(PCA9685_PRESCALE, prescale);
    delay(5);
    write8(PCA9685_MODE1, oldmode);                  // wake up this way (datasheet recommended)
    delay(5);
    write8(PCA9685_MODE1, oldmode | 0xA1);           // restart + enable auto-increment
}

void MyPCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t reg = 0x06 + 4 * channel;
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(on & 0xFF);
    Wire.write(on >> 8);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    Wire.endTransmission();
}

void MyPCA9685::setPin(uint8_t channel, uint16_t val, bool invert) {
    if (val > 4095) val = 4095;
    if (invert) {
        setPWM(channel, 4095 - val, 0);
    } else {
        setPWM(channel, 0, val);
    }
}

PiperPico2::PiperPico2()
  : _pwm(MyPCA9685(0x40)),
    _display(128, 32, &Wire, -1),
    _enc1(20, 19, 21, 22),
    _enc2(16, 17, 10, 18)
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        _sensorMin[i] = 0;
        _sensorMax[i] = 4095;
    }
}

bool PiperPico2::begin() {
    Serial.begin(115200);
    delay(1800);  // หน่วงเพื่อ stability ของ I2C bus
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();
    delay(50);

    initPins();
    initPWM();
    initOLED();
    playTone(2200, 120); delay(30);
    playTone(2500, 120); delay(30);
    playTone(3000, 180);

    loadCalibration();
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
}

void PiperPico2::initPWM() {
    _pwm.begin();

    // สำคัญมาก: ตั้งความถี่ PWM เป็น 50 Hz สำหรับเซอร์โว (มาตรฐาน)
    // ถ้า servo ของคุณชอบ 60 Hz สามารถเปลี่ยนเป็น 60 ได้
    _pwm.setPWMFreq(50);

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t ch = SERVO_CHANNELS[i];
        _pwm.setPWM(ch, 0, 0);  // ปิด servo เริ่มต้น (หรือ set เป็นกลาง: map(90,0,180,SERVO_MIN,SERVO_MAX))
    }
}

// ส่วนที่เหลือเหมือนเดิม (initOLED, motor, setServo, playTone, adcRead, calibrate ฯลฯ)
// คุณสามารถ copy ส่วนเหล่านี้จากโค้ดเดิมของคุณมา paste ต่อจากนี้

void PiperPico2::initOLED() {
    if (!_display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        return;
    }
    _display.clearDisplay();
    _display.setTextSize(2);
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(10, 0);
    //_display.println(F("PIPER"));
    //_display.println(F("ROBOTECH"));
    _display.display();
    delay(100);
}

void PiperPico2::setMotorPWMFrequency(uint32_t freqHz) {
    if (freqHz < 1000 || freqHz > 50000) {
        freqHz = 15000;
    }
    analogWriteFreq(freqHz);
}

void PiperPico2::motor(char id, int speed) {
    speed = constrain(speed, -100, 100);

    uint8_t pwm_pin = 0;
    uint8_t in1_ch = getMotorIn1(id);
    uint8_t in2_ch = getMotorIn2(id);

    if (in1_ch == 255) return;

    uint16_t pwmDuty = 0;
    bool in1 = LOW, in2 = LOW;

    switch (id) {
        case 'A': pwm_pin = PWM_PIN_A; break;
        case 'B': pwm_pin = PWM_PIN_B; break;
        case 'C': pwm_pin = PWM_PIN_C; break;
        case 'D': pwm_pin = PWM_PIN_D; break;
        default: return;
    }

    if (speed > 0) {
        pwmDuty = (uint16_t)(speed * SPEED_SCALE);
        in1 = HIGH; in2 = LOW;
    } else if (speed < 0) {
        pwmDuty = (uint16_t)(-speed * SPEED_SCALE);
        in1 = LOW; in2 = HIGH;
    } else {
        pwmDuty = 0;
        in1 = LOW; in2 = LOW;
    }

    analogWrite(pwm_pin, pwmDuty);
    _pwm.setPin(in1_ch, in1 ? 4095 : 0);
    _pwm.setPin(in2_ch, in2 ? 4095 : 0);
}

uint8_t PiperPico2::getMotorIn1(char id) {
    switch (id) {
        case 'A': return 0;
        case 'B': return 5;
        case 'C': return 2;
        case 'D': return 6;
        default: return 255;
    }
}

uint8_t PiperPico2::getMotorIn2(char id) {
    switch (id) {
        case 'A': return 1;
        case 'B': return 4;
        case 'C': return 3;
        case 'D': return 7;
        default: return 255;
    }
}

void PiperPico2::setServo(uint8_t logicalChannel, int angle) {
    if (logicalChannel > 4) return;

    uint8_t realChannel = SERVO_CHANNELS[logicalChannel];

    angle = constrain(angle, 0, 180);
    uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    _pwm.setPWM(realChannel, 0, pulse);
}

void PiperPico2::playTone(uint16_t freq, uint16_t duration_ms) {
    if (freq == 0 || duration_ms == 0) return;

    unsigned long half_period = 500000UL / freq;
    unsigned long cycles = (unsigned long)duration_ms * freq / 1000UL;

    for (unsigned long i = 0; i < cycles; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delayMicroseconds(half_period);
        digitalWrite(BUZZER_PIN, LOW);
        delayMicroseconds(half_period);
    }
    digitalWrite(BUZZER_PIN, LOW);
}

uint16_t PiperPico2::adcRead(uint8_t channel) {
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

    return (adcMin(sensor) + adcMax(sensor))/2;
}

uint16_t PiperPico2::readLine(uint8_t sensor) {
    if (sensor >= NUM_SENSORS) return 0;
    return adcRead(sensor);
}

void PiperPico2::readLines(uint16_t* rawValues) {
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

void PiperPico2::calibrate() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        _sensorMin[i] = 4095;
        _sensorMax[i] = 0;
    }

    unsigned long start = millis();
    unsigned long lastToneTime = 0;  // เวลาที่เล่นเสียงครั้งล่าสุด

    while (millis() - start < 10000) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            uint16_t val = adcRead(i);
            if (val < _sensorMin[i]) _sensorMin[i] = val;
            if (val > _sensorMax[i]) _sensorMax[i] = val;
        }

        // แทรกเสียงติ๊งเบา ๆ ทุก 1.5 วินาที (1500 ms)
        if (millis() - lastToneTime >= 1000) {
            playTone(1200, 60);  // เสียงเบา 1200 Hz นาน 60 ms
            lastToneTime = millis();
        }

        delay(50);
    }

    saveCalibration();
    
    // เสียงยืนยันจบการ calibrate (เหมือนเดิม)
    playTone(2500, 100); delay(100);
    playTone(2500, 100); delay(300);
    playTone(3000, 500); delay(100);
}

bool PiperPico2::loadCalibration() {
    uint8_t buffer[4];
    for (int i = 0; i < NUM_SENSORS; i++) {
        uint16_t addr = EEPROM_BASE_ADDR + (i * 4);
        eepromRead(addr, buffer, 4);
        _sensorMin[i] = (buffer[1] << 8) | buffer[0];
        _sensorMax[i] = (buffer[3] << 8) | buffer[2];
    }
    return true;
}

bool PiperPico2::saveCalibration() {
    uint8_t buffer[4];
    for (int i = 0; i < NUM_SENSORS; i++) {
        buffer[0] = _sensorMin[i] & 0xFF;
        buffer[1] = _sensorMin[i] >> 8;
        buffer[2] = _sensorMax[i] & 0xFF;
        buffer[3] = _sensorMax[i] >> 8;
        uint16_t addr = EEPROM_BASE_ADDR + (i * 4);
        eepromWrite(addr, buffer, 4);
    }
    return true;
}

void PiperPico2::eepromWrite(uint16_t ee_addr, const uint8_t* data, uint8_t len) {
    Wire.beginTransmission(EEPROM_I2C_ADDR);
    Wire.write((ee_addr >> 8) & 0xFF);
    Wire.write(ee_addr & 0xFF);
    for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
    Wire.endTransmission();
    delay(5);
}

void PiperPico2::eepromRead(uint16_t ee_addr, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(EEPROM_I2C_ADDR);
    Wire.write((ee_addr >> 8) & 0xFF);
    Wire.write(ee_addr & 0xFF);
    Wire.endTransmission();
    Wire.requestFrom(EEPROM_I2C_ADDR, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        data[i] = Wire.read();
    }
}

float PiperPico2::getVoltage() {
    uint16_t adc_value = adcRead(11);
    return (adc_value / 4095.0f) * 3.3f * VOLTAGE_DIVIDER_RATIO;
}

void PiperPico2::showVoltageUntilButton() {
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
        delay(100);
    }
    playTone(3000, 500);
    delay(30);
}

float PiperPico2::knopRead() {
    uint16_t knop_value = map(adcRead(10), 0, 2800, 0, 1203);
    return knop_value ;
}

void PiperPico2::resetEncoders() {
    _enc1.resetEncoders();
    _enc2.resetEncoders();
}