#include "my_BMI160.h"

// กำหนดค่าคงที่ static
const float my_BMI160::GYRO_THRESHOLD     = 0.3f;
const float my_BMI160::ALPHA              = 0.95f;
const float my_BMI160::ACCEL_FILTER_ALPHA = 0.1f;

// Static variables
unsigned long my_BMI160::_lastTime     = 0;
float my_BMI160::_angleX               = 0.0f;
float my_BMI160::_angleY               = 0.0f;
float my_BMI160::_angleZ               = 0.0f;
float my_BMI160::_gyroOffsetX          = 0.0f;
float my_BMI160::_gyroOffsetY          = 0.0f;
float my_BMI160::_gyroOffsetZ          = 0.0f;
float my_BMI160::_accelX_prev          = 0.0f;
float my_BMI160::_accelY_prev          = 0.0f;
float my_BMI160::_accelZ_prev          = 0.0f;

// Constructor
my_BMI160::my_BMI160(uint8_t address) 
  : _address(address), _wire(nullptr) {}

// ตั้งค่า I2C bus
void my_BMI160::setWire(TwoWire& wire) {
  _wire = &wire;
}

// Initialize sensor
bool my_BMI160::begin() {
  if (_wire == nullptr) {
    _wire = &Wire;  // fallback ถ้ายังไม่ได้ตั้งค่า
  }

  _wire->begin();
  _wire->setClock(400000);

  // Check chip ID
  _wire->beginTransmission(_address);
  _wire->write(my_BMI160_CHIP_ID);
  _wire->endTransmission();
  _wire->requestFrom(_address, (uint8_t)1);
  if (_wire->available()) {
    uint8_t chipID = _wire->read();
    if (chipID != 0xD1) {
      return false;
    }
  } else {
    return false;
  }

  // Reset sensor
  writeRegister(my_BMI160_CMD, 0xB6);
  delay(100);

  // Configure accelerometer (±2g, ODR 400Hz)
  writeRegister(my_BMI160_ACCEL_CONF, 0x2A);
  writeRegister(my_BMI160_ACCEL_RANGE, 0x03);

  // Configure gyroscope (±2000 dps, ODR 400Hz)
  writeRegister(my_BMI160_GYRO_CONF, 0x2A);
  writeRegister(my_BMI160_GYRO_RANGE, 0x00);

  // Start sensors
  writeRegister(my_BMI160_CMD, 0x11);
  delay(50);
  writeRegister(my_BMI160_CMD, 0x15);
  delay(50);

  if (!calibrateGyro()) {
    return false;
  }

  _lastTime = micros();
  return true;
}

// Read angles (เหมือนเดิม)
void my_BMI160::readAngles(float &roll, float &pitch, float &yaw) {
  int16_t ax, ay, az, gx, gy, gz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0f;
  float accelY = ay / 16384.0f;
  float accelZ = az / 16384.0f;

  accelX = ACCEL_FILTER_ALPHA * accelX + (1.0f - ACCEL_FILTER_ALPHA) * _accelX_prev;
  accelY = ACCEL_FILTER_ALPHA * accelY + (1.0f - ACCEL_FILTER_ALPHA) * _accelY_prev;
  accelZ = ACCEL_FILTER_ALPHA * accelZ + (1.0f - ACCEL_FILTER_ALPHA) * _accelZ_prev;
  _accelX_prev = accelX;
  _accelY_prev = accelY;
  _accelZ_prev = accelZ;

  float gyroX = (gx / 16.4f) - _gyroOffsetX;
  float gyroY = (gy / 16.4f) - _gyroOffsetY;
  float gyroZ = (gz / 16.4f) - _gyroOffsetZ;

  const float GYRO_CALIBRATION_ALPHA = 0.01f;
  if (abs(gx / 16.4f) < GYRO_THRESHOLD) {
    _gyroOffsetX = GYRO_CALIBRATION_ALPHA * (gx / 16.4f) + (1.0f - GYRO_CALIBRATION_ALPHA) * _gyroOffsetX;
  }
  if (abs(gy / 16.4f) < GYRO_THRESHOLD) {
    _gyroOffsetY = GYRO_CALIBRATION_ALPHA * (gy / 16.4f) + (1.0f - GYRO_CALIBRATION_ALPHA) * _gyroOffsetY;
  }
  if (abs(gz / 16.4f) < GYRO_THRESHOLD) {
    _gyroOffsetZ = GYRO_CALIBRATION_ALPHA * (gz / 16.4f) + (1.0f - GYRO_CALIBRATION_ALPHA) * _gyroOffsetZ;
  }

  if (abs(gyroX) < GYRO_THRESHOLD) gyroX = 0.0f;
  if (abs(gyroY) < GYRO_THRESHOLD) gyroY = 0.0f;
  if (abs(gyroZ) < GYRO_THRESHOLD) gyroZ = 0.0f;

  float accelRoll  = atan2(accelY, accelZ) * 180.0f / PI;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0f / PI;

  unsigned long currentTime = micros();
  float deltaTime = (currentTime - _lastTime) / 1000000.0f;
  if (deltaTime > 0.1f) deltaTime = 0.1f;
  _lastTime = currentTime;

  float gyroAngleX = _angleX + gyroX * deltaTime;
  float gyroAngleY = _angleY + gyroY * deltaTime;
  float gyroAngleZ = _angleZ + gyroZ * deltaTime;

  _angleX = ALPHA * gyroAngleX + (1.0f - ALPHA) * accelRoll;
  _angleY = ALPHA * gyroAngleY + (1.0f - ALPHA) * accelPitch;
  _angleZ = gyroAngleZ;

  while (_angleZ > 180.0f)  _angleZ -= 360.0f;
  while (_angleZ < -180.0f) _angleZ += 360.0f;

  roll  = _angleX;
  pitch = _angleY;
  yaw   = _angleZ;
}

float my_BMI160::gyro(char axis) {
  float roll, pitch, yaw;
  readAngles(roll, pitch, yaw);
  switch (axis) {
    case 'x': case 'X': return roll;
    case 'y': case 'Y': return pitch;
    case 'z': case 'Z': return yaw;
    default: return 0.0f;
  }
}

void my_BMI160::resetYaw() {
  _angleZ = 0.0f;
}

void my_BMI160::resetAngles() {
  _angleX = 0.0f;
  _angleY = 0.0f;
  _angleZ = 0.0f;
}

void my_BMI160::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

void my_BMI160::readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
  _wire->beginTransmission(_address);
  _wire->write(my_BMI160_GYRO_DATA);
  _wire->endTransmission(false);
  _wire->requestFrom(_address, (uint8_t)12);

  if (_wire->available() == 12) {
    *gx = (int16_t)(_wire->read() | (_wire->read() << 8));
    *gy = (int16_t)(_wire->read() | (_wire->read() << 8));
    *gz = (int16_t)(_wire->read() | (_wire->read() << 8));
    *ax = (int16_t)(_wire->read() | (_wire->read() << 8));
    *ay = (int16_t)(_wire->read() | (_wire->read() << 8));
    *az = (int16_t)(_wire->read() | (_wire->read() << 8));
  } else {
    *ax = *ay = *az = *gx = *gy = *gz = 0;
  }
}

bool my_BMI160::calibrateGyro() {
  const int CALIBRATION_SAMPLES = 600;
  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
  float sumAX = 0.0f, sumAY = 0.0f, sumAZ = 0.0f;
  float varX = 0.0f, varY = 0.0f, varZ = 0.0f;
  int16_t gx, gy, gz;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t ax, ay, az;
    readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
    float gX = gx / 16.4f;
    float gY = gy / 16.4f;
    float gZ = gz / 16.4f;
    float aX = ax / 16384.0f;
    float aY = ay / 16384.0f;
    float aZ = az / 16384.0f;
    sumX += gX; sumY += gY; sumZ += gZ;
    sumAX += aX; sumAY += aY; sumAZ += aZ;
    varX += gX * gX; varY += gY * gY; varZ += gZ * gZ;
    delay(5);
  }

  float meanX = sumX / CALIBRATION_SAMPLES;
  float meanY = sumY / CALIBRATION_SAMPLES;
  float meanZ = sumZ / CALIBRATION_SAMPLES;
  varX = varX / CALIBRATION_SAMPLES - meanX * meanX;
  varY = varY / CALIBRATION_SAMPLES - meanY * meanY;
  varZ = varZ / CALIBRATION_SAMPLES - meanZ * meanZ;

  if (varX > 0.1f || varY > 0.1f || varZ > 0.1f) {
    return false;
  }

  _gyroOffsetX = meanX;
  _gyroOffsetY = meanY;
  _gyroOffsetZ = meanZ;
  _accelX_prev = sumAX / CALIBRATION_SAMPLES;
  _accelY_prev = sumAY / CALIBRATION_SAMPLES;
  _accelZ_prev = sumAZ / CALIBRATION_SAMPLES;
  return true;
}

void my_BMI160::recalibrateGyro() {
  const int CALIBRATION_SAMPLES = 50;
  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
  float sumAX = 0.0f, sumAY = 0.0f, sumAZ = 0.0f;
  float varX = 0.0f, varY = 0.0f, varZ = 0.0f;
  int16_t gx, gy, gz;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t ax, ay, az;
    readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
    float gX = gx / 16.4f;
    float gY = gy / 16.4f;
    float gZ = gz / 16.4f;
    float aX = ax / 16384.0f;
    float aY = ay / 16384.0f;
    float aZ = az / 16384.0f;
    sumX += gX; sumY += gY; sumZ += gZ;
    sumAX += aX; sumAY += aY; sumAZ += aZ;
    varX += gX * gX; varY += gY * gY; varZ += gZ * gZ;
    delay(2);
  }

  float meanX = sumX / CALIBRATION_SAMPLES;
  float meanY = sumY / CALIBRATION_SAMPLES;
  float meanZ = sumZ / CALIBRATION_SAMPLES;
  varX = varX / CALIBRATION_SAMPLES - meanX * meanX;
  varY = varY / CALIBRATION_SAMPLES - meanY * meanY;
  varZ = varZ / CALIBRATION_SAMPLES - meanZ * meanZ;

  _gyroOffsetX = meanX;
  _gyroOffsetY = meanY;
  _gyroOffsetZ = meanZ;
  _accelX_prev = sumAX / CALIBRATION_SAMPLES;
  _accelY_prev = sumAY / CALIBRATION_SAMPLES;
  _accelZ_prev = sumAZ / CALIBRATION_SAMPLES;
}

void reset_gyro160(my_BMI160& gyro) {
  gyro.resetAngles();
}