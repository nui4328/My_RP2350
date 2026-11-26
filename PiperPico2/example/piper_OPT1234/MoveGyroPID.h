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
int pulsesPerRevolution_FW  = 210;
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
int bw_to_rotate = 130;

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

//-------------------------->>>
bool _offset;

/////////////////////////////////////////////////////////////////////////////////

float PULSES_PER_CM = 180.00f;

// ==================== Global PID สำหรับเดิน ====================
float move_kp = 1.45f;
float move_ki = 0.015f;
float move_kd = 0.042f;

float last_PULSES_up = 0.0f;

// ==================== Global Position ====================
float current_position_cm = 0.0f;

// Overshoot
const float UP_OVERSHOOT_CM   = 0.20f;
const float DOWN_OVERSHOOT_CM = 0.65f;

// Safety
#define MAX_SAFE_ARM_HEIGHT   27.0f
#define ARM_STUCK_TIME_MS     180
#define ARM_MIN_PULSES        4

// ==================== Global Arm Variables ====================
bool arm_moving = false;
bool arm_homing_mode = false;
int  arm_direction = 0;
long arm_goal_pulses = 0;
unsigned long arm_last_time = 0;
float last_arm_command = -999.0f;

unsigned long arm_last_move_time = 0;
long arm_last_pulses_stuck = 0;

int _target_cm = 1;

enum ArmHomingState {
  HOMING_DOWN,
  HOMING_BACKUP
};
ArmHomingState homing_state = HOMING_DOWN;
unsigned long homing_backup_start = 0;

// ====================== ตั้งค่า ======================
void set_PULSES_updown(float cm) { PULSES_PER_CM = cm; }
void set_move_pid(float kp, float ki, float kd) {
    move_kp = kp; move_ki = ki; move_kd = kd;
}
////////////////////////////////////////////////////////////////////////////////////////


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
void Set_distanceBW_to_turnGyro(int dis) {
  bw_to_rotate = dis;
}

void Set_distanceFW_to_turnGyro(int dis) {
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

void turnGyro(int speed, float degree)   ////////////////////////////////////////////////////////////////////////////////////
{
  if (fabs(degree) < 0.6f) return;

  int absSpeed = constrain(abs(speed), 38, 98);   // เพิ่มความเร็วสูงสุด
  bool isRight = (degree > 0);

  delay(25);

  const int BACKUP_SPEED = 25;

  // ========== Backup ==========
  encoder1.resetEncoders();
  if ((ch_line && _move_fw) || _set_f) {
    do { 
      robot.motor('A', -BACKUP_SPEED); 
      robot.motor('C', -BACKUP_SPEED); 
    } while (abs(encoder1.Poss_L()) < bw_to_rotate);
    robot.motor('A', 20); robot.motor('C', 20); delay(30);
  } 
  else if ((ch_line && _move_bw) || _set_b) {
    do { 
      robot.motor('A', BACKUP_SPEED); 
      robot.motor('C', BACKUP_SPEED); 
    } while (abs(encoder1.Poss_L()) < fw_to_rotate);
    robot.motor('A', -20); robot.motor('C', -20); delay(30);
  } 
  else {
    robot.motor('A', -10); robot.motor('C', 10); delay(20);
  }

  robot.motor('A', 0); robot.motor('C', 0);
  delay(200);

  // ========== Gyro Setup ==========
  gyro.resetAngles();
  float initialAngle = averageGyroZ(55);
  float targetAngle = initialAngle + degree;

  integral = 0.0f;
  lastError = 0.0f;

  unsigned long startTime = millis();
  unsigned long lastTime = millis();
  unsigned long rampStart = millis();

  int stableCount = 0;
  const float TOLERANCE_TURN = 0.55f;

  float derivFilter = 0.0f;
  const float alpha = 0.68f;

  while (true) {
    float currentAngle = gyro.gyro('z');
    float error = constrainAngle180(targetAngle - currentAngle);
    float absError = fabs(error);

    unsigned long now = millis();
    float dt = constrain((now - lastTime) / 1000.0f, 0.003f, 0.08f);
    lastTime = now;

    // Integral ปานกลาง
    integral *= 0.972f;
    integral += error * dt * 1.12f;
    integral = constrain(integral, -155.0f, 155.0f);

    float derivative = (error - lastError) / dt;
    derivFilter = alpha * derivative + (1.0f - alpha) * derivFilter;
    lastError = error;

    float kp = isRight ? r_kp : l_kp;
    float ki = isRight ? r_ki : l_ki;
    float kd = isRight ? r_kd : l_kd;

    float output = kp * error + ki * integral + kd * derivFilter * 1.18f;

    // ====================== Ramp: แรงก่อน แล้วค่อยชะลอ ======================
    
    float elapsed = now - rampStart;
    
    float rampFactor = 1.0f;
    
    // 1. ออกตัวแรงเร็ว (แรงตั้งแต่เริ่ม)
    if (elapsed < 35) {                    // 65ms แรก
      rampFactor = 0.65f + (elapsed / 65.0f) * 0.45f;   // เริ่ม 0.65 → พุ่งขึ้นเร็ว
    }
    // 2. วิ่งด้วยความเร็วสูง (cruise)
    else if (absError > 45.0f) {
      rampFactor = 1.0f;
    }
    // 3. ชะลอเมื่อใกล้เป้า
    else {
      // ไม่ต้องทำอะไร rampFactor ยังเป็น 1.0 แล้วค่อยใช้ slowDownFactor
    }

    // Ramp Down เมื่อใกล้เป้า (ชะลอแรง)
    float slowDownFactor = 1.0f;
    if (absError < 35.0f) {
      slowDownFactor = 0.26f + (absError / 35.0f) * 0.64f;   // ช้าลงเร็ว
    } else if (absError < 65.0f) {
      slowDownFactor = 0.68f;
    }

    float currentMaxSpeed = absSpeed * rampFactor * slowDownFactor;

    // Near target → ลด gain
    if (absError < 12.0f) {
      output *= 0.40f + (absError / 12.0f) * 0.60f;
    }

    // Anti-windup
    if (output > currentMaxSpeed) {
      integral -= (output - currentMaxSpeed) * dt * 1.18f;
      output = currentMaxSpeed;
    } else if (output < -currentMaxSpeed) {
      integral -= (output + currentMaxSpeed) * dt * 1.18f;
      output = -currentMaxSpeed;
    }

    // Bias
    float left_bias  = isRight ? turnR_left_bias  : turnL_left_bias;
    float right_bias = isRight ? turnR_right_bias : turnL_right_bias;

    int leftMotor  = (int)(output * left_bias);
    int rightMotor = (int)(-output * right_bias);

    leftMotor  = constrain(leftMotor,  -100, 100);
    rightMotor = constrain(rightMotor, -100, 100);

    robot.motor('A', leftMotor);
    robot.motor('C', rightMotor);

    // Stable Condition
    if (absError <= TOLERANCE_TURN || fabs(output) <= 5.5f) {
      stableCount++;
      if (stableCount >= 16) {
        int brake = isRight ? -11 : 11;
        robot.motor('A', brake);
        robot.motor('C', -brake);
        delay(16);
        robot.motor('A', 0);
        robot.motor('C', 0);
        delay(15);
        break;
      }
    } else {
      stableCount = 0;
    }

    if (millis() - startTime > 1500) {
      robot.motor('A', 0); robot.motor('C', 0);
      Serial.println("TURN SMOOTH TIMEOUT");
      break;
    }

    delay(3);
  }
  _offset = true;  //--->> คืนค่ารีเซนไจโร เพื่อเซตให้ตรง
}


#endif  // TURNGYROPID_H