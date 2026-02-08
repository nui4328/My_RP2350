#ifndef TURNGYROPID_H
#define TURNGYROPID_H

#include <PiperPico2.h>
#include <my_BMI160.h>

// ประกาศ extern สำหรับ object ที่สร้างใน .ino หลัก
extern PiperPico2 robot;
extern my_BMI160 gyro;
extern EncoderLibrarys encoder1;
extern EncoderLibraryss encoder2;

// ค่าคงที่สำหรับ encoder
const float wheelDiameter = 4.0f;                      // cm
const float wheelCircumference = PI * wheelDiameter;
const int pulsesPerRevolution = 20;
const float pulsesPerCm = pulsesPerRevolution / wheelCircumference;
// ความเร็วฐาน (cm/s) เมื่อแบตเต็ม (เช่น วัดจริงตอนแบต 8.4V หรือ 12V)
const float baseSpeed_cm_per_s = 110.0f;       // ← สำคัญ! ต้องวัดจริง เช่น วิ่ง 100cm ใช้กี่วินาที แล้วคำนวณ

// แรงดันแบตเต็ม (หน่วย V) ที่ baseSpeed ถูกวัด
const float fullVoltage = 12.6f;               // ปรับตามแบตของคุณ เช่น 2S LiPo = 8.4V, 3S=12.6V

// ค่าสัมประสิทธิ์ชดเชย (ยิ่งแบตต่ำ ยิ่งเพิ่ม PWM)
const float voltageCompensationFactor = 1.15f; // ปรับได้ 1.1~1.3 ตามการทดสอบ

// ตัวแปรสถานะต่าง ๆ (ประกาศจริงที่นี่เพื่อหลีกเลี่ยง linker error)
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
int fw_to_rotate = 50;  // pulse สำหรับถอยออกจากเส้น (ปรับได้)

// PID สำหรับหมุน (ปรับ aggressive ขึ้นนิดสำหรับ BNO085)
float r_kp = 2.6f, r_ki = 0.030f, r_kd = 16.0f;   // หมุนขวา
float l_kp = 2.6f, l_ki = 0.030f, l_kd = 16.0f;   // หมุนซ้าย

// PID สำหรับเดินตรง (ใหม่)
float straight_kp = 1.8f;
float straight_ki = 0.015f;
float straight_kd = 12.0f;

// ค่าคงที่หลัก
const float TOLERANCE_TURN     = 1.1f;     // องศา สำหรับหมุน
const float TOLERANCE_STRAIGHT = 1.8f;     // องศา สำหรับเดินตรง
const unsigned long TIMEOUT_MS = 500;     // ms (1.8 วินาที)

// ตัวแปร PID static
static float integral = 0.0f;
static float lastError = 0.0f;

// ฟังก์ชันช่วย
float constrainAngle180(float angle) {
  while (angle > 180.0f)  angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float averageGyroZ(int samples) {
  float sum = 0.0f;
  for (int i = 0; i < samples; i++) {
    sum += gyro.gyro('z');
    delay(2);
  }
  return sum / static_cast<float>(samples);
}

// ฟังก์ชันตั้งค่า
void Set_travel_distance_toturnGyro(int dis) {
  fw_to_rotate = dis;
}

void set_pid_turnR(float kp, float ki, float kd) {
  r_kp = kp; r_ki = ki; r_kd = kd;
}

void set_pid_turnL(float kp, float ki, float kd) {
  l_kp = kp; l_ki = ki; l_kd = kd;
}

void set_pid_straight(float kp, float ki, float kd) {
  straight_kp = kp; straight_ki = ki; straight_kd = kd;
}

// ────────────────────────────────────────────────
// ฟังก์ชันหมุนด้วย gyro (ปรับปรุงแล้ว)
// ────────────────────────────────────────────────
void turnGyro(int speed, float degree) {
  int absSpeed = constrain(abs(speed), 70, 100);
  bool isRight = (degree > 0);

  if (abs(degree) < 0.5f) return;

  delay(50);

  const int BACKUP_SPEED = 15;

  // ถอยออกจากเส้น (ใช้ตัวแปรที่ประกาศจริงแล้ว)
  bool needForwardBackup  = (ch_line && _move_fw) || _set_f;
  bool needBackwardBackup = (ch_line && _move_bw) || _set_b;

  if (needForwardBackup || needBackwardBackup) {
    encoder1.resetEncoders();

    if (needForwardBackup) {
      do {
        robot.motor('A', -BACKUP_SPEED); robot.motor('C', -BACKUP_SPEED);
      } while (encoder1.Poss_R() > -fw_to_rotate);

      robot.motor('A', 10); robot.motor('C', 10); delay(40);
      robot.motor('A', -1); robot.motor('C', -1); delay(100);
    }
    else if (needBackwardBackup) {
      do {
        robot.motor('A', BACKUP_SPEED); robot.motor('C', BACKUP_SPEED);
      } while (encoder1.Poss_R() < fw_to_rotate);

      robot.motor('A', -10); robot.motor('C', -10); delay(40);
      robot.motor('A', 1); robot.motor('C', 1); delay(100);
    }
  }
  else {
    robot.motor('A', -5); robot.motor('C', 5); delay(20);
    robot.motor('A', 0); robot.motor('C', 0); delay(10);
  }

  // เบรกก่อนหมุน
  robot.motor('A', 0); robot.motor('C', 0);
  delay(40);

  // Reset PID & Gyro
  integral = 0.0f;
  lastError = 0.0f;
  gyro.resetAngles();
  gyro.recalibrateGyro();
  delay(40);

  float initialAngle = averageGyroZ(12);
  float targetAngle = initialAngle - degree;

  float error = 0.0f;
  float output = 0.0f;
  unsigned long lastTime = millis();
  unsigned long startTime = millis();
  int stableCount = 0;

  const float rampUpTimeMs = 100.0f;
  unsigned long rampStart = millis();

  while (true) {
    float currentAngle = gyro.gyro('z');
    error = constrainAngle180(targetAngle + currentAngle);  // แก้เป็น target - current

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    dt = constrain(dt, 0.001f, 0.04f);

    integral += error * dt;
    integral = constrain(integral, -140.0f, 140.0f);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float kp = isRight ? r_kp : l_kp;
    float ki = isRight ? r_ki : l_ki;
    float kd = isRight ? r_kd : l_kd;

    output = kp * error + ki * integral + kd * derivative;

const float DEADBAND = 1.0f;          // ขยายนิดหน่อย
const float FADE_FACTOR = 0.65f;       // ไม่ลดแรง (หรือลอง 0.9f)
const float MIN_OUTPUT = 7.0f;        // ขั้นต่ำเพื่อให้มอเตอร์ยังหมุนต่อ

if (abs(error) < DEADBAND) {
    float scale = (abs(error) / DEADBAND) * FADE_FACTOR;
    output *= scale;
    
    // เพิ่มขั้นต่ำถ้า output ใกล้ 0 แต่ยังมี error
    if (abs(output) < MIN_OUTPUT && abs(error) > 0.5f) {
        output = (output >= 0) ? MIN_OUTPUT : -MIN_OUTPUT;
    }
} 

    unsigned long elapsed = now - rampStart;
    float currentMaxSpeed = (elapsed < rampUpTimeMs) ? absSpeed * (elapsed / rampUpTimeMs) : absSpeed;

    // Anti-windup
    if (output > currentMaxSpeed || output < -currentMaxSpeed) {
      integral -= error * dt * 0.6f;
      integral = constrain(integral, -140.0f, 140.0f);
    }

    output = constrain(output, -currentMaxSpeed, currentMaxSpeed);

    int leftMotor  = (int)-output;
    int rightMotor = (int)output;
    robot.motor('A', leftMotor);
    robot.motor('C', rightMotor);

    if (abs(error) <= TOLERANCE_TURN && abs(output) <= 2.0f) {
      stableCount++;
      int brakeLeft, brakeRight;
      if (isRight) {
          brakeLeft  = -10;   // เบรกขวาแรงขึ้น
          brakeRight = 10;
      } else {
          brakeLeft  = 10;
          brakeRight = -10;   // เบรกซ้ายแรงขึ้น
      }
      robot.motor('A', brakeLeft);
      robot.motor('C', brakeRight);
      delay(10);
      robot.motor('A', 0);
      robot.motor('C', 0);
      delay(20);
      break;
    } else {
      stableCount = 0;
    }

    if (millis() - startTime > TIMEOUT_MS) {
      robot.motor('A', 0); robot.motor('C', 0);
      Serial.println("!!! TURN TIMEOUT !!!");
      break;
    }

    delay(4);
  }

  float actual = constrainAngle180(gyro.gyro('z') - initialAngle);
  delay(20);
}

// ────────────────────────────────────────────────
// เดินหน้า / ถอยหลัง ด้วย gyro ช่วยรักษาทิศทาง + encoder วัดระยะ
// ────────────────────────────────────────────────
void moveStraight(int baseSpeed, float distance_cm, bool forward = true) {
  if (distance_cm <= 0) return;

  int dir = forward ? 1 : -1;
  int absSpeed = constrain(abs(baseSpeed), 50, 100);

  encoder1.resetEncoders();

  integral = 0.0f;
  lastError = 0.0f;
  gyro.resetAngles();

  float initialAngle = averageGyroZ(10);
  float targetAngle = initialAngle;  // เดินตรง → target = initial

  float targetPulses = distance_cm * pulsesPerCm * dir;
  float errorPulses = 0.0f;
  float errorAngle = 0.0f;
  float outputAngle = 0.0f;

  unsigned long lastTime = millis();
  unsigned long startTime = millis();
  int stableCount = 0;

  const float rampUpTimeMs = 150.0f;
  unsigned long rampStart = millis();

  while (true) {
    float currentAngle = gyro.gyro('z');
    errorAngle = constrainAngle180(targetAngle - currentAngle);

    float currentPulses = forward ? encoder1.Poss_R() : -encoder1.Poss_R();
    errorPulses = targetPulses - currentPulses;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    dt = constrain(dt, 0.001f, 0.05f);

    integral += errorAngle * dt;
    integral = constrain(integral, -80.0f, 80.0f);

    float derivative = (errorAngle - lastError) / dt;
    lastError = errorAngle;

    outputAngle = straight_kp * errorAngle + straight_ki * integral + straight_kd * derivative;

    if (abs(errorAngle) < 0.8f) outputAngle *= 0.4f;

    unsigned long elapsed = now - rampStart;
    float currentMaxSpeed = (elapsed < rampUpTimeMs) ? absSpeed * (elapsed / rampUpTimeMs) : absSpeed;

    int baseMotor = dir * currentMaxSpeed;
    int correction = (int)outputAngle;

    int left  = baseMotor + correction;
    int right = baseMotor - correction;

    left  = constrain(left,  -100, 100);
    right = constrain(right, -100, 100);

    robot.motor('A', left);
    robot.motor('C', right);

    if (abs(errorPulses) < 20 && abs(errorAngle) <= TOLERANCE_STRAIGHT && abs(outputAngle) <= 4.0f) {
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

#endif  // TURNGYROPID_H