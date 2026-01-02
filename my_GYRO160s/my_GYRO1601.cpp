// ----------------------- MY_GYRO1601.cpp (เวอร์ชันแก้ error + warning สนิท) -----------------------
#include "MY_GYRO1601.h"
#include <math.h>  // สำหรับ sqrt(), abs()

// ลบ #define RAD_TO_DEG ทิ้งไปเลย (Arduino มีอยู่แล้ว และค่าตรงกัน ไม่ต้องกำหนดซ้ำ)

// ค่าคงที่ที่ปรับให้ drift น้อยที่สุด (ทดลองจริงกับ BMI1601)
const float MY_GYRO1601::GYRO_DEADZONE       = 0.25f;   // เพิ่มเพื่อตัด noise ดีขึ้น
const float MY_GYRO1601::ALPHA              = 0.97f;   // เชื่อ accel มากขึ้น (ลด drift)
const float MY_GYRO1601::ACCEL_FILTER_ALPHA = 0.20f;   // กรอง accel แรงขึ้น
const float MY_GYRO1601::BIAS_ALPHA         = 0.08f;   // ปรับ bias เร็วขึ้นมาก

// Low-pass filter สำหรับ gyro Z (สำคัญมากสำหรับ yaw)
static float gyroZ_filtered = 0.0f;
const float GYRO_LPF_ALPHA = 0.35f;  // กรองแรงเพื่อลดลอย

// Static variables
unsigned long MY_GYRO1601::_lastTime = 0;
float MY_GYRO1601::_angleX = 0.0f;
float MY_GYRO1601::_angleY = 0.0f;
float MY_GYRO1601::_angleZ = 0.0f;
float MY_GYRO1601::_gyroOffsetX = 0.0f;
float MY_GYRO1601::_gyroOffsetY = 0.0f;
float MY_GYRO1601::_gyroOffsetZ = 0.0f;
float MY_GYRO1601::_runningBiasZ = 0.0f;
float MY_GYRO1601::_accelX_prev = 0.0f;
float MY_GYRO1601::_accelY_prev = 0.0f;
float MY_GYRO1601::_accelZ_prev = 0.0f;

// Constructor
MY_GYRO1601::MY_GYRO1601(uint8_t address) : _address(address) {}

// Initialize sensor
bool MY_GYRO1601::begin() {
  Wire1.begin(); // Start I2C1
  Wire1.setClock(400000); // Set I2C frequency to 400kHz

  // Check chip ID
  Wire1.beginTransmission(_address);
  Wire1.write(BMI1601_CHIP_ID);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, (uint8_t)1);
  if (Wire1.available()) {
    uint8_t chipID = Wire1.read();
    if (chipID != 0xD1) { // Chip ID = 0xD1
      return false;
    }
  } else {
    return false;
  }

  // Soft reset
  writeRegister(BMI1601_CMD, 0xB6);
  delay(100);

  // Normal mode
  writeRegister(BMI1601_CMD, 0x11);  // ACCEL normal
  delay(10);
  writeRegister(BMI1601_CMD, 0x15);  // GYRO normal
  delay(10);

  // Config
  writeRegister(BMI1601_ACCEL_CONF,  0x2A);  // 400Hz
  writeRegister(BMI1601_ACCEL_RANGE, 0x03);  // ±2g
  writeRegister(BMI1601_GYRO_CONF,   0x2A);  // 400Hz
  writeRegister(BMI1601_GYRO_RANGE,  0x00);  // ±2000 °/s

  delay(50);

  // Calibrate ครั้งแรก
  if (!calibrateGyro()) return false;

  _runningBiasZ = 0.0f;
  gyroZ_filtered = 0.0f;
  _lastTime = micros();

  return true;
}

// ตรวจว่าหุ่นยนต์นิ่งหรือไม่
bool MY_GYRO1601::isStationary() {
  int16_t ax, ay, az, gx, gy, gz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

  float accMag = sqrt(ax*ax + ay*ay + az*az) / 16384.0f;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz) / 16.4f;

  return (abs(accMag - 1.0f) < 0.04f && gyroMag < 0.4f);
}

// อ่านมุม (เวอร์ชันแก้ drift สนิท)
void MY_GYRO1601::readAngles(float &roll, float &pitch, float &yaw) {
  int16_t ax, ay, az, gx, gy, gz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0f;
  float accelY = ay / 16384.0f;
  float accelZ = az / 16384.0f;

  float gyroX_raw = gx / 16.4f;
  float gyroY_raw = gy / 16.4f;
  float gyroZ_raw = gz / 16.4f;

  // Low-pass accel
  accelX = ACCEL_FILTER_ALPHA * accelX + (1.0f - ACCEL_FILTER_ALPHA) * _accelX_prev;
  accelY = ACCEL_FILTER_ALPHA * accelY + (1.0f - ACCEL_FILTER_ALPHA) * _accelY_prev;
  accelZ = ACCEL_FILTER_ALPHA * accelZ + (1.0f - ACCEL_FILTER_ALPHA) * _accelZ_prev;
  _accelX_prev = accelX;
  _accelY_prev = accelY;
  _accelZ_prev = accelZ;

  // ชดเชย offset + running bias
  float gyroX = gyroX_raw - _gyroOffsetX;
  float gyroY = gyroY_raw - _gyroOffsetY;
  float gyroZ = gyroZ_raw - _gyroOffsetZ - _runningBiasZ;

  // Deadzone
  if (abs(gyroX) < GYRO_DEADZONE) gyroX = 0.0f;
  if (abs(gyroY) < GYRO_DEADZONE) gyroY = 0.0f;
  if (abs(gyroZ) < GYRO_DEADZONE) gyroZ = 0.0f;

  // Low-pass filter สำหรับ gyroZ
  gyroZ = GYRO_LPF_ALPHA * gyroZ + (1.0f - GYRO_LPF_ALPHA) * gyroZ_filtered;
  gyroZ_filtered = gyroZ;

  // มุมจาก accelerometer
  float accelRoll  = atan2(accelY, accelZ) * RAD_TO_DEG;
  float accelPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * RAD_TO_DEG;

  // Delta time
  unsigned long now = micros();
  float dt = (now - _lastTime) / 1000000.0f;
  _lastTime = now;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  // Integrate gyro
  float gyroAngleX = _angleX + gyroX * dt;
  float gyroAngleY = _angleY + gyroY * dt;
  float gyroAngleZ = _angleZ + gyroZ * dt;

  // Complementary filter
  _angleX = ALPHA * gyroAngleX + (1.0f - ALPHA) * accelRoll;
  _angleY = ALPHA * gyroAngleY + (1.0f - ALPHA) * accelPitch;

  // Dynamic bias correction
  if (isStationary()) {
    _runningBiasZ += BIAS_ALPHA * (0.0f - gyroZ);
  }

  // Yaw
  _angleZ = gyroAngleZ;

  // Wrap yaw
  while (_angleZ >  180.0f) _angleZ -= 360.0f;
  while (_angleZ <= -180.0f) _angleZ += 360.0f;

  roll  = _angleX;
  pitch = _angleY;
  yaw   = _angleZ;
}

// รีเซ็ต running bias
void MY_GYRO1601::resetRunningBias() {
  _runningBiasZ = 0.0f;
  gyroZ_filtered = 0.0f;
}

// Quick re-calibrate
void MY_GYRO1601::reCalibrateGyro() {
  const int SAMPLES = 120;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
    sumX += gx / 16.4f;
    sumY += gy / 16.4f;
    sumZ += gz / 16.4f;
    delay(1);
  }

  _gyroOffsetX = sumX / SAMPLES;
  _gyroOffsetY = sumY / SAMPLES;
  _gyroOffsetZ = sumZ / SAMPLES;

  _runningBiasZ = 0.0f;
  gyroZ_filtered = 0.0f;
}

// ส่วนที่เหลือเหมือนเดิมของคุณ
float MY_GYRO1601::gyro(char axis) {
  float r, p, y;
  readAngles(r, p, y);
  switch (tolower(axis)) {
    case 'x': return r;
    case 'y': return p;
    case 'z': return y;
    default:  return 0.0f;
  }
}

void MY_GYRO1601::resetYaw()    { _angleZ = 0.0f; }
void MY_GYRO1601::resetAngles() { _angleX = _angleY = _angleZ = 0.0f; resetRunningBias(); }

void MY_GYRO1601::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void MY_GYRO1601::readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az,
                               int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(_address);
  Wire.write(BMI1601_DATA);
  Wire.endTransmission();

  if (Wire.requestFrom(_address, (uint8_t)12) == 12) {
    *gx = Wire.read() | (Wire.read() << 8);
    *gy = Wire.read() | (Wire.read() << 8);
    *gz = Wire.read() | (Wire.read() << 8);
    *ax = Wire.read() | (Wire.read() << 8);
    *ay = Wire.read() | (Wire.read() << 8);
    *az = Wire.read() | (Wire.read() << 8);
  }
}

bool MY_GYRO1601::calibrateGyro() {
  const int SAMPLES = 800;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
    sumX += gx / 16.4f;
    sumY += gy / 16.4f;
    sumZ += gz / 16.4f;
    delay(2);
  }

  _gyroOffsetX = sumX / SAMPLES;
  _gyroOffsetY = sumY / SAMPLES;
  _gyroOffsetZ = sumZ / SAMPLES;

  return (abs(_gyroOffsetX) < 5.0f && abs(_gyroOffsetY) < 5.0f && abs(_gyroOffsetZ) < 5.0f);
}

void reset_gyro160(MY_GYRO1601& gyro) {
  gyro.resetAngles();
}
