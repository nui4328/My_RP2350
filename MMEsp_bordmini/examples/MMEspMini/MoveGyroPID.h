#ifndef TURNGYROPID_H
#define TURNGYROPID_H

#include "MMEsp_bordmini.h"
#include "my_GYRO1601.h"

// ประกาศ extern สำหรับ object ที่สร้างใน .ino หลัก
extern MMEsp_bordmini board;
extern my_GYRO1601 gyro;

// ค่าคงที่สำหรับ encoder (ปรับตามที่คุณบอก)
const float wheelDiameter = 4.1;                      // เส้นผ่านศูนย์กลางล้อ (cm)
const float wheelCircumference = PI * wheelDiameter;  // เส้นรอบวงล้อ (cm)
const int pulsesPerRevolution = 250;                  // จำนวนพัลส์ต่อรอบ (ปรับใหม่ตามจริง)
const float pulsesPerCm = pulsesPerRevolution / wheelCircumference; // พัลส์ต่อเซนติเมตร

// ตัวแปรสถานะต่าง ๆ
bool lines = false;
bool lines_fw = false;
bool lines_bw = false;
bool sett_f = false;
bool ch_lr = false;
bool ch_set_fb = false;
bool ch_bw = false;
bool line_l = true;
bool line_r = true;
char ch_lrs;
bool set_bb = false;
int motor_slow = 18;
int fw_to_rotate = 180;

// PID แยกซ้าย-ขวา
float r_kp = 1.8f, r_ki = 0.012f, r_kd = 0.45f;   // หมุนขวา
float l_kp = 1.7f, l_ki = 0.010f, l_kd = 0.40f;   // หมุนซ้าย

const float TOLERANCE = 2.5f;
const unsigned long TIMEOUT_MS = 500;

// ตัวแปร PID
static float integral = 0.0f;
static float lastError = 0.0f;

// ฟังก์ชันช่วย
float averageGyroZ(int samples) {
  float sum = 0.0f;
  for (int i = 0; i < samples; i++) {
    sum += gyro.gyro('z');
    delay(2);
  }
  return sum / static_cast<float>(samples);
}

float constrainAngle180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void set_pid_moveR(float kp, float ki, float kd) {
  r_kp = kp; r_ki = ki; r_kd = kd;
}

void set_pid_moveL(float kp, float ki, float kd) {
  l_kp = kp; l_ki = ki; l_kd = kd;
}

// ฟังก์ชันหลัก: speed มาก่อน degree
// degree > 0 = หมุนขวา, degree < 0 = หมุนซ้าย เสมอ
// speed ติดลบ = กลับความเร็วเป็นบวกเท่านั้น (ไม่พลิกทิศทาง)
/// ฟังก์ชันหลัก: speed มาก่อน degree
void turnGyro(int speed, float degree) {
  int absSpeed = constrain(abs(speed), 70, 100);  // เร่งสูงสุด
  bool isRight = (degree > 0);
  gyro.recalibrateGyro();
  if (abs(degree) < 0.5f) {;
    return;
  }

  // Reset PID
  integral = 0.0f;
  lastError = 0.0f;

  gyro.resetAngles();

  float initialAngle = averageGyroZ(30);
  float targetAngle = initialAngle + degree;

  float error = 0.0f;
  float output = 0.0f;
  unsigned long lastTime = millis();
  unsigned long startTime = millis();
  int stableCount = 0;

  // Ramp-up เร็วสุด (แทบไม่ชะลอตอนเริ่ม)
  float currentMaxSpeed = 0.0f;
  const float rampUpTimeMs = 100.0f;  // 100ms → เร่งเต็มสปีดเร็วมาก
  unsigned long rampStart = millis();


  while (true) {
    float currentAngle = gyro.gyro('z');
    error = targetAngle - currentAngle;
    error = constrainAngle180(error);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt > 0.08f) dt = 0.02f;
    if (dt < 0.001f) dt = 0.001f;

    integral += error * dt;
    integral = constrain(integral, -80.0f, 80.0f);

    float derivative = (error - lastError) / dt;
    lastError = error;

    // PID เร่งเต็มที่ (เข้าเป้าเร็วมาก)
    float kp = isRight ? r_kp : l_kp;
    float ki = isRight ? r_ki : l_ki;
    float kd = isRight ? r_kd : l_kd;

    output = kp * error + ki * integral + kd * derivative;

    // Deadband ต่ำมากเพื่อเข้าเป้าเต็มสปีด
    if (abs(error) < 0.9f) {
      output = 0.0f;
    }

    // Ramp-up
    unsigned long elapsed = now - rampStart;
    if (elapsed < rampUpTimeMs) {
      currentMaxSpeed = absSpeed * (elapsed / rampUpTimeMs);
    } else {
      currentMaxSpeed = absSpeed;
    }

    output = constrain(output, -currentMaxSpeed, currentMaxSpeed);

    // ส่งมอเตอร์
    int leftMotor  = (int)output;
    int rightMotor = (int)-output;
    board.Motor(leftMotor, rightMotor);

    // เงื่อนไขหยุดเร็วขึ้น
    if (abs(error) <= TOLERANCE && abs(output) <= 4.0f) {
      stableCount++;
      if (stableCount >= 10) {  // หยุดเร็ว ~180ms
        int brakeValue = isRight ? -18 : 18;  // เบรกแรงขึ้นนิดเพื่อหยุดชัด
        board.Motor(brakeValue, -brakeValue);
        delay(18);
        board.Motor(0, 0);
        break;
      }
    } else {
      stableCount = 0;
    }

    if (millis() - startTime > TIMEOUT_MS) {
      board.Motor(0, 0);
      Serial.println("!!! TIMEOUT !!!");
      break;
    }


    delay(5);  // loop ถี่ขึ้นมาก
  }

  float actual = constrainAngle180(gyro.gyro('z') - initialAngle);;
  gyro.recalibrateGyro();
}

#endif  // TURNGYROPID_H