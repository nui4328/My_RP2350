#ifndef MMEsp_bordmini_h
#define MMEsp_bordmini_h

#include <Arduino.h>
#include <Wire.h>

#define NUM_SENSORS 10
#define SAMPLES_PER_READ 10
#define EEPROM_ADDRESS 0x50

class MMEsp_bordmini {
public:
  MMEsp_bordmini();

  void begin();

  // Line Sensor
  uint16_t adcRead(uint8_t channel);
  void readRaw(uint16_t values[NUM_SENSORS]);
  void readCalibrated(uint16_t values[NUM_SENSORS]);
  void readLine(uint16_t values[NUM_SENSORS], bool onoff = false);

  uint16_t adc_min(uint8_t index);
  uint16_t adc_max(uint8_t index);
  uint16_t adc_md(uint8_t index);

  // Calibration
  void resetCalibration();
  void updateCalibration(uint8_t index, uint16_t value);

  // Motor
  void Motor(int leftSpeed, int rightSpeed);

  // Servo
  void servo(uint8_t channel, uint16_t degree);
  void servoDetach(uint8_t channel);

  // Calibration data - public
  uint16_t sensorMin[NUM_SENSORS];
  uint16_t sensorMax[NUM_SENSORS];
  uint16_t sensorValues[NUM_SENSORS];

  void loadCalibrationFromEEPROM();
  void saveCalibrationToEEPROM();

private:
  static const uint8_t MUX_S0 = 38;
  static const uint8_t MUX_S1 = 40;
  static const uint8_t MUX_S2 = 37;
  static const uint8_t MUX_S3 = 39;
  static const uint8_t MUX_Z  = 9;

  static const uint8_t PWMA  = 33;
  static const uint8_t AIN1  = 35;
  static const uint8_t AIN2  = 34;
  static const uint8_t PWMB  = 18;
  static const uint8_t BIN1  = 14;
  static const uint8_t BIN2  = 17;

  static const uint8_t SERVO_PINS[3];

  void _servoWrite(uint8_t realPin, uint16_t degree);
  void _writeWordToEEPROM(uint16_t addr, uint16_t data);
  uint16_t _readWordFromEEPROM(uint16_t addr);
};

#endif