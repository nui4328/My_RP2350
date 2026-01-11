#include "MMEsp_bordmini.h"

const uint8_t MMEsp_bordmini::SERVO_PINS[3] = {42, 41, 1};

MMEsp_bordmini::MMEsp_bordmini() {
  resetCalibration();
}

void MMEsp_bordmini::begin() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  loadCalibrationFromEEPROM();
  Motor(0, 0);
}

// adcRead
uint16_t MMEsp_bordmini::adcRead(uint8_t channel) {
  digitalWrite(MUX_S0, channel & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
  digitalWrite(MUX_S3, (channel >> 3) & 1);
  delayMicroseconds(300);

  uint32_t sum = 0;
  for (uint8_t i = 0; i < SAMPLES_PER_READ; i++) {
    sum += analogRead(MUX_Z);
    delayMicroseconds(30);
  }
  return sum / SAMPLES_PER_READ;
}

// readRaw, readCalibrated, readLine
void MMEsp_bordmini::readRaw(uint16_t values[NUM_SENSORS]) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    values[i] = adcRead(i);
  }
}

void MMEsp_bordmini::readCalibrated(uint16_t values[NUM_SENSORS]) {
  readRaw(values);
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorMax[i] - sensorMin[i] < 50) {
      values[i] = 500;
    } else {
      values[i] = map(values[i], sensorMin[i], sensorMax[i], 0, 1000);
      values[i] = constrain(values[i], 0, 1000);
    }
  }
}

void MMEsp_bordmini::readLine(uint16_t values[NUM_SENSORS], bool onoff) {
  readCalibrated(values);
  if (!onoff) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      values[i] = (values[i] > 500) ? 1000 : 0;
    }
  }
}

// min / max
uint16_t MMEsp_bordmini::adc_min(uint8_t index) {
  if (index >= NUM_SENSORS) return 4095;
  return sensorMin[index];
}

uint16_t MMEsp_bordmini::adc_max(uint8_t index) {
  if (index >= NUM_SENSORS) return 0;
  return sensorMax[index];
}

uint16_t MMEsp_bordmini::adc_md(uint8_t index) {
  if (index >= NUM_SENSORS) return 0;
  return (sensorMax[index]+sensorMax[index])/2;
}
// Calibration
void MMEsp_bordmini::resetCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }
}

void MMEsp_bordmini::updateCalibration(uint8_t index, uint16_t value) {
  if (index >= NUM_SENSORS) return;
  if (value < sensorMin[index]) sensorMin[index] = value;
  if (value > sensorMax[index]) sensorMax[index] = value;
}

// Motor
void MMEsp_bordmini::Motor(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -100, 100);
  rightSpeed = constrain(rightSpeed, -100, 100);

  int lpwm = map(abs(leftSpeed), 0, 100, 0, 255);
  int rpwm = map(abs(rightSpeed), 0, 100, 0, 255);

  digitalWrite(AIN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(AIN2, leftSpeed < 0 ? HIGH : LOW);
  ledcWrite(PWMA, leftSpeed == 0 ? 0 : lpwm);

  digitalWrite(BIN1, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(BIN2, rightSpeed < 0 ? HIGH : LOW);
  ledcWrite(PWMB, rightSpeed == 0 ? 0 : rpwm);
}

// Servo
void MMEsp_bordmini::servo(uint8_t channel, uint16_t degree) {
  degree = constrain(degree, 0, 180);
  if (channel < 16 || channel > 18) return;
  uint8_t index = channel - 16;
  uint8_t realPin = SERVO_PINS[index];
  _servoWrite(realPin, degree);
}

void MMEsp_bordmini::_servoWrite(uint8_t realPin, uint16_t degree) {
  ledcAttach(realPin, 50, 14);
  uint32_t duty = map(degree, 0, 180, 160, 2050);
  ledcWrite(realPin, duty);
}

void MMEsp_bordmini::servoDetach(uint8_t channel) {
  if (channel < 16 || channel > 18) return;
  uint8_t realPin = SERVO_PINS[channel - 16];
  ledcDetach(realPin);
}

// EEPROM
void MMEsp_bordmini::_writeWordToEEPROM(uint16_t addr, uint16_t data) {
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((addr >> 8) & 0xFF);
  Wire.write(addr & 0xFF);
  Wire.write(data & 0xFF);
  Wire.write((data >> 8) & 0xFF);
  Wire.endTransmission();
  delay(5);
}

uint16_t MMEsp_bordmini::_readWordFromEEPROM(uint16_t addr) {
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((addr >> 8) & 0xFF);
  Wire.write(addr & 0xFF);
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDRESS, 2);
  uint16_t data = 0;
  if (Wire.available() >= 2) {
    data = Wire.read();
    data |= (Wire.read() << 8);
  }
  return data;
}

void MMEsp_bordmini::loadCalibrationFromEEPROM() {
  uint16_t addr = 0;
  bool valid = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint16_t mn = _readWordFromEEPROM(addr); addr += 2;
    uint16_t mx = _readWordFromEEPROM(addr); addr += 2;
    if (mn >= mx || mn < 100 || mx > 4000) valid = false;
    sensorMin[i] = mn;
    sensorMax[i] = mx;
  }
  if (valid) Serial.println("Loaded calibration from EEPROM");
  else Serial.println("Invalid EEPROM -> Calibrate new");
}

void MMEsp_bordmini::saveCalibrationToEEPROM() {
  uint16_t addr = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    _writeWordToEEPROM(addr, sensorMin[i]); addr += 2;
    _writeWordToEEPROM(addr, sensorMax[i]); addr += 2;
  }
  Serial.println("Calibration SAVED to EEPROM!");
}