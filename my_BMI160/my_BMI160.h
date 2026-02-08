#ifndef my_BMI160_H
#define my_BMI160_H

#include <Wire.h>

#ifndef PI
#define PI 3.141592653589793
#endif

class my_BMI160 {
public:
  static const uint8_t my_BMI160_CHIP_ID     = 0x00;  // Register for chip ID
  static const uint8_t my_BMI160_CMD         = 0x7E;  // Command register
  static const uint8_t my_BMI160_ACCEL_CONF  = 0x40;  // Accelerometer config
  static const uint8_t my_BMI160_ACCEL_RANGE = 0x41;  // Accelerometer range
  static const uint8_t my_BMI160_GYRO_CONF   = 0x42;  // Gyroscope config
  static const uint8_t my_BMI160_GYRO_RANGE  = 0x43;  // Gyroscope range
  static const uint8_t my_BMI160_GYRO_DATA   = 0x0C;  // Data register start

  // Constants
  static const float GYRO_THRESHOLD;
  static const float ALPHA;
  static const float ACCEL_FILTER_ALPHA;

  // Constructor
  my_BMI160(uint8_t address = 0x69);

  // เมธอดสำคัญ: เลือกใช้ Wire หรือ Wire1 (หรือ TwoWire อื่น)
  void setWire(TwoWire& wire);

  bool begin();
  void readAngles(float &roll, float &pitch, float &yaw);
  float gyro(char axis);
  void resetYaw();
  void resetAngles();
  void recalibrateGyro();
  bool calibrate();
  void recalibrate();


private:
  uint8_t _address;
  TwoWire* _wire = nullptr;  // Pointer ไปยัง I2C bus ที่เลือกใช้

  static unsigned long _lastTime;
  static float _angleX;
  static float _angleY;
  static float _angleZ;
  static float _gyroOffsetX;
  static float _gyroOffsetY;
  static float _gyroOffsetZ;
  static float _accelX_prev;
  static float _accelY_prev;
  static float _accelZ_prev;

  void writeRegister(uint8_t reg, uint8_t value);
  void readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
  bool calibrateGyro();
};

// ฟังก์ชันภายนอก (ถ้าต้องการใช้)
void reset_gyro160(my_BMI160& gyro);

#endif