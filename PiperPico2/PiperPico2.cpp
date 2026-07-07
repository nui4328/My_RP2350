#include "PiperPico2.h"

PiperPico2::PiperPico2()
  : _pwm(Adafruit_PWMServoDriver(0x40)),  // I2C address 0x40 (default ของ PCA9685)
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
    delay(800);                     // ลดจาก 1800 → พอสำหรับ Serial stability

    // === I2C Bus Recovery (เคลียร์ bus ที่อาจค้าง) ===
    Wire.setSDA(4);
    Wire.setSCL(5);
    
    // ตั้งเป็น GPIO ชั่วคราวเพื่อ clock pulses
    pinMode(4, OUTPUT);  // SDA
    pinMode(5, OUTPUT);  // SCL
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    delay(500);
    
  

    // ส่ง 9-10 clock pulses เพื่อเคลียร์ slave ที่อาจค้าง
    for (int i = 0; i < 10; i++) {
        digitalWrite(5, LOW);
        delayMicroseconds(5);
        digitalWrite(5, HIGH);
        delayMicroseconds(15);
    }

    // ปล่อย bus เป็น input (pull-up จะดึง HIGH)
    pinMode(4, INPUT);
    pinMode(5, INPUT);
    delay(100);

    // เริ่ม I2C ใหม่
    Wire.begin();
    Wire.setClock(100000);        // แนะนำ 800 kHz (เสถียรกว่า 1 MHz ในทางปฏิบัติ)

    // === เริ่มต้นส่วนอื่น ๆ ===
    initPins();
    initPWM();      // ต้องเรียกหลัง Wire.setClock แล้ว
    initOLED();
    
       // เสียงเริ่มต้น
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
    _pwm.begin();                     // เริ่มไลบรารี Adafruit
    Wire.setClock(100000);            // เริ่มที่ 400 kHz (เสถียร + เร็วพอสมควร)
    // ถ้ายังมี noise หรือ servo สั่น → ลดเหลือ 200000 หรือ 100000
    _pwm.setPWMFreq(50);              // ← เปลี่ยนเป็น 50 Hz (มาตรฐานสำหรับ servo analog)
    delay(3);                        // ให้ PCA9685 ปรับความถี่เสร็จก่อน
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t ch = SERVO_CHANNELS[i];
        _pwm.setPWM(ch, 0, 4096);     // Full OFF → ปิด servo ถูกต้อง
    }
}

void PiperPico2::setservo(uint8_t logicalChannel, int angle) {
    if (logicalChannel > 6) return;
    uint8_t realChannel = SERVO_CHANNELS[logicalChannel];
    angle = constrain(angle, 0, 180);
    uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    _pwm.setPWM(realChannel, 0, pulse);
}

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
    _pwm.setPWM(in1_ch, 0, in1 ? 4095 : 0);
    _pwm.setPWM(in2_ch, 0, in2 ? 4095 : 0);
}

uint8_t PiperPico2::getMotorIn1(char id) {
    switch (id) {
        case 'A': return 1;
        case 'B': return 5;
        case 'C': return 2;
        case 'D': return 6;
        default: return 255;
    }
}

uint8_t PiperPico2::getMotorIn2(char id) {
    switch (id) {
        case 'A': return 0;
        case 'B': return 4;
        case 'C': return 3;
        case 'D': return 7;
        default: return 255;
    }
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
    return (adcMin(sensor) + adcMax(sensor)) / 2;
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
    unsigned long lastToneTime = 0;

    while (millis() - start < 10000) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            uint16_t val = adcRead(i);
            if (val < _sensorMin[i]) _sensorMin[i] = val;
            if (val > _sensorMax[i]) _sensorMax[i] = val;
        }

        if (millis() - lastToneTime >= 1000) {
            playTone(1200, 60);
            lastToneTime = millis();
        }

        delay(50);
    }

    saveCalibration();
    
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
    return knop_value;
}

void PiperPico2::resetEncoders() {
    _enc1.resetEncoders();
    _enc2.resetEncoders();
}