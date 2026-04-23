#ifndef TURNGYROPID_H
#define TURNGYROPID_H

#include <Arduino.h>   // เพื่อใช้ constrain, fabs, millis ฯลฯ

// ประกาศ extern (เหมือนเดิม)
extern PiperPico2 robot;
extern my_BMI160 gyro;
extern EncoderLibrarys encoder1;
extern EncoderLibraryss encoder2;

// ==================== ค่าคงที่ encoder ====================
const float wheelDiameter = 4.1f;
const float wheelCircumference = PI * wheelDiameter;

const int pulsesPerRevolution     = 210;   // ค่าเฉลี่ยทั่วไป
int pulsesPerRevolution_FW  = 220;
int pulsesPerRevolution_BW  = 220;

const float pulsesPerCm     = pulsesPerRevolution     / wheelCircumference;
const float pulsesPerCm_fw  = pulsesPerRevolution_FW  / wheelCircumference;
const float pulsesPerCm_bw  = pulsesPerRevolution_BW  / wheelCircumference;

// ตัวแปรสถานะ (เหมือนเดิม)
bool ch_line = false;
bool _move_fw = false;
bool _move_bw = false;
bool _set_f = false;
bool _set_b = false;
bool ch_lr = false;
bool ch_set_fb = false;
bool ch_bw = false;
bool line_l = true;
bool line_r = true;
char ch_lrs;
int motor_slow = 18;
int fw_to_rotate = 130;

// ==================== PID ====================
// สำหรับหมุน
float r_kp = 2.8f, r_ki = 0.035f, r_kd = 18.0f;
float l_kp = 2.6f, l_ki = 0.030f, l_kd = 16.5f;

// สำหรับเดินตรง
float straight_kp = 1.8f;
float straight_ki = 0.015f;
float straight_kd = 12.0f;

// Motor bias สำหรับหมุน (แยกซ้าย-ขวา)
float turnL_left_bias   = 1.00f;
float turnL_right_bias  = 1.00f;
float turnR_left_bias   = 1.00f;
float turnR_right_bias  = 1.00f;

// ค่าคงที่หลัก
const float TOLERANCE_TURN     = 2.4f;
const float TOLERANCE_STRAIGHT = 1.8f;
const unsigned long TIMEOUT_MS = 1000;

// ตัวแปร PID static
static float integral = 0.0f;
static float lastError = 0.0f;

void set_PULSES_moveFW( int cm)
    {
        pulsesPerRevolution_FW = cm;
    }

void set_PULSES_moveBW( int cm)
    {
        pulsesPerRevolution_BW = cm;
    }
// ==================== ฟังก์ชันช่วย ====================
float constrainAngle180(float angle) {
  while (angle > 180.0f)  angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float averageGyroZ(int samples) {
  float sum = 0.0f;
  for (int i = 0; i < samples; i++) {
    sum += gyro.gyro('z');
    delay(3);
  }
  return sum / static_cast<float>(samples);
}

// ==================== ฟังก์ชันตั้งค่า ====================
void Set_travel_distance_to_turnGyro(int dis) {
  fw_to_rotate = dis;
}

void set_pid_turnL(float kp, float ki, float kd) {
  l_kp = kp; l_ki = ki; l_kd = kd;
}

void set_pid_turnR(float kp, float ki, float kd) {
  r_kp = kp; r_ki = ki; r_kd = kd;
}

void set_pid_straight(float kp, float ki, float kd) {
  straight_kp = kp; straight_ki = ki; straight_kd = kd;
}

void set_turnL_motor_bias(float left_bias, float right_bias) {
  turnL_left_bias  = constrain(left_bias,  0.70f, 1.35f);
  turnL_right_bias = constrain(right_bias, 0.70f, 1.35f);
}

void set_turnR_motor_bias(float left_bias, float right_bias) {
  turnR_left_bias  = constrain(left_bias,  0.70f, 1.35f);
  turnR_right_bias = constrain(right_bias, 0.70f, 1.35f);
}

// ==================== turnGyro (เหมือนเดิมแต่ปรับเล็กน้อยให้เสถียร) ====================
void turnGyro(int speed, float degree) {
  if (fabs(degree) < 0.5f) return;

  int absSpeed = constrain(abs(speed), 70, 100);
  bool isRight = (degree > 0);

  delay(30);

  const int BACKUP_SPEED = 30;

  bool needForwardBackup  = (ch_line && _move_fw) || _set_f;
  bool needBackwardBackup = (ch_line && _move_bw) || _set_b;

  encoder1.resetEncoders();

  if (needForwardBackup) {
    do {
      robot.motor('A', -BACKUP_SPEED);
      robot.motor('C', -BACKUP_SPEED);
    } while (encoder1.Poss_L() > -(fw_to_rotate));
    robot.motor('A', 25); robot.motor('C', 25); delay(15);
  } 
  else if (needBackwardBackup) {
    do {
      robot.motor('A', BACKUP_SPEED);
      robot.motor('C', BACKUP_SPEED);
    } while (encoder1.Poss_L() < fw_to_rotate);
    robot.motor('A', -35); robot.motor('C', -35); delay(25);
  } 
  else {
    robot.motor('A', -8); robot.motor('C', 8); delay(15);
  }

  robot.motor('A', 0); robot.motor('C', 0);
  delay(10);

  integral = 0.0f;
  lastError = 0.0f;
  gyro.resetAngles();
   // ---------- GYRO SETUP ----------

  float initialAngle = averageGyroZ(25);
  float targetAngle = initialAngle + degree;

  unsigned long lastTime = millis();
  unsigned long startTime = millis();
  unsigned long rampStart = millis();
  int stableCount = 0;

  const float rampUpTimeMs = 80.0f;

  while (true) {
    float currentAngle = gyro.gyro('z');
    float error = constrainAngle180(targetAngle - currentAngle);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    dt = constrain(dt, 0.001f, 0.05f);

    integral += error * dt;
    integral = constrain(integral, -180.0f, 180.0f);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float kp = isRight ? r_kp : l_kp;
    float ki = isRight ? r_ki : l_ki;
    float kd = isRight ? r_kd : l_kd;

    float output = kp * error + ki * integral + kd * derivative;

    if (fabs(error) < 3.0f) {
      output *= 0.65f + (fabs(error) / 3.0f) * 0.35f;
    }

    float currentMaxSpeed = (now - rampStart < rampUpTimeMs) ?
                            absSpeed * ((now - rampStart) / rampUpTimeMs) : absSpeed;

    if (output > currentMaxSpeed) {
      integral -= (output - currentMaxSpeed) * dt * 0.8f;
      output = currentMaxSpeed;
    } else if (output < -currentMaxSpeed) {
      integral -= (output + currentMaxSpeed) * dt * 0.8f;
      output = -currentMaxSpeed;
    }

    float left_bias  = isRight ? turnR_left_bias  : turnL_left_bias;
    float right_bias = isRight ? turnR_right_bias : turnL_right_bias;

    int leftMotor  = (int)(output * left_bias);
    int rightMotor = (int)(-output * right_bias);

    leftMotor  = constrain(leftMotor,  -100, 100);
    rightMotor = constrain(rightMotor, -100, 100);

    robot.motor('A', leftMotor);
    robot.motor('C', rightMotor);

    if (fabs(error) <= TOLERANCE_TURN && fabs(output) <= 3.0f) {
      stableCount++;
      if (stableCount >= 7) {
        int brake = isRight ? -18 : 18;
        robot.motor('A', brake);
        robot.motor('C', -brake);
        delay(30);
        robot.motor('A', 0);
        robot.motor('C', 0);
        delay(10);
        break;
      }
    } else {
      stableCount = 0;
    }

    if (millis() - startTime > TIMEOUT_MS) {
      robot.motor('A', 0); robot.motor('C', 0);
      Serial.println("!!! TURN TIMEOUT !!!");
      break;
    }

    delay(3);
  }
}

// ==================== moveStraight (ปรับให้ใช้ pulses fw/bw) ====================
void moveStraight(int baseSpeed, float distance_cm, bool forward = true) {
  if (distance_cm <= 0) return;

  int dir = forward ? 1 : -1;
  int absSpeed = constrain(abs(baseSpeed), 50, 100);

  float pulsesPerCm_used = forward ? pulsesPerCm_fw : pulsesPerCm_bw;

  encoder1.resetEncoders();
  integral = 0.0f;
  lastError = 0.0f;
  gyro.resetAngles();

  float initialAngle = averageGyroZ(10);
  float targetAngle = initialAngle;

  float targetPulses = distance_cm * pulsesPerCm_used * dir;

  unsigned long lastTime = millis();
  unsigned long startTime = millis();
  unsigned long rampStart = millis();
  int stableCount = 0;

  const float rampUpTimeMs = 150.0f;

  while (true) {
    float currentAngle = gyro.gyro('z');
    float errorAngle = constrainAngle180(targetAngle - currentAngle);

    float currentPulses = forward ? encoder1.Poss_L() : -encoder1.Poss_L();
    float errorPulses = targetPulses - currentPulses;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    dt = constrain(dt, 0.001f, 0.05f);

    integral += errorAngle * dt;
    integral = constrain(integral, -80.0f, 80.0f);

    float derivative = (errorAngle - lastError) / dt;
    lastError = errorAngle;

    float outputAngle = straight_kp * errorAngle + straight_ki * integral + straight_kd * derivative;

    if (fabs(errorAngle) < 0.8f) outputAngle *= 0.4f;

    unsigned long elapsed = now - rampStart;
    float currentMaxSpeed = (elapsed < rampUpTimeMs) ? 
                            absSpeed * (elapsed / rampUpTimeMs) : absSpeed;

    int baseMotor = dir * currentMaxSpeed;
    int correction = (int)outputAngle;

    int left  = constrain(baseMotor + correction, -100, 100);
    int right = constrain(baseMotor - correction, -100, 100);

    robot.motor('A', left);
    robot.motor('C', right);

    if (fabs(errorPulses) < 20 && fabs(errorAngle) <= TOLERANCE_STRAIGHT && fabs(outputAngle) <= 4.0f) {
      stableCount++;
      if (stableCount >= 6) {
        int brake = forward ? -12 : 12;
        robot.motor('A', brake);
        robot.motor('C', brake);
        delay(30);
        robot.motor('A', 0);
        robot.motor('C', 0);
        delay(40);
        break;
      }
    } else {
      stableCount = 0;
    }

    if (millis() - startTime > TIMEOUT_MS) {
      robot.motor('A', 0); robot.motor('C', 0);
      Serial.println("!!! MOVE TIMEOUT !!!");
      break;
    }

    delay(5);
  }

  gyro.recalibrateGyro();
  delay(20);
}

void moveForward(int speed, float distance_cm) {
  moveStraight(speed, distance_cm, true);
}

void moveBackward(int speed, float distance_cm) {
  moveStraight(speed, distance_cm, false);
}

// ==================== WAIT BUTTON ====================
void sw()
{
  Serial.println("Waiting for button...");

  while (true)
  {
    if (digitalRead(2) == 0)
    {
      while (digitalRead(2) == 0);   // debounce
      delay(50);
      break;
    }
    delay(10);
  }

  robot.playTone(3000, 200);
  Serial.println("Start!");
}

#endif  // TURNGYROPID_H