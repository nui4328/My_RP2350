#ifndef TURNGYROPID_H
#define TURNGYROPID_H

#include <Arduino.h>   // เพื่อใช้ constrain, fabs, millis ฯลฯ
#include <BNO055.h>

BNO055 imu;

// ประกาศ extern (เหมือนเดิม)
extern PiperPico2 robot;
extern my_BMI160 gyro;
extern EncoderLibrarys encoder1;
extern EncoderLibraryss encoder2;

// ==================== ค่าคงที่ encoder ====================
const float wheelDiameter = 4.1f;
const float wheelCircumference = PI * wheelDiameter;

const int pulsesPerRevolution     = 210;   // ค่าเฉลี่ยทั่วไป
int pulsesPerRevolution_FW  = 190;    ////////////////////////////////////////////////////////>>>>ตั้งค่าระยะทาง
int pulsesPerRevolution_BW  = 195;

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
int degree_turnGyro;

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
    imu.update();
    sum += imu.yaw();
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


void turnGyro(int speed, float degree, bool absolute = false)
{
    // ========================================================
    // CONFIGURATION
    // ========================================================

    const int maxSpeed =
        constrain(abs(speed), 40, 95);

    // อยู่ในช่วงนี้ให้หยุดและยอมรับตำแหน่ง
    const float STOP_ERROR = 2.2f;

    // เขตแก้ตำแหน่งแบบเบา
    const float CORRECTION_ZONE = 6.0f;

    // ต้องอยู่ในเขตหยุดต่อเนื่อง
    const uint8_t STOP_STABLE_COUNT = 8;

    // กำลังขั้นต่ำตอนหมุนปกติ
    const int MIN_POWER_NORMAL = 22;

    // กำลังตอนแก้กลับใกล้เป้าหมาย
    const int CORRECTION_POWER = 20;

    // กำลังสูงสุดตอนแก้กลับ
    const int CORRECTION_MAX_POWER = 21;

    const float KD_FILTER = 0.85f;

    const float MOVE_DETECTION_ANGLE = 0.15f;
    const unsigned long STUCK_TIME = 300UL;

    const int BACKUP_SPEED = 25;

    // ========================================================
    // BACKUP
    // ========================================================

    encoder1.resetEncoders();

    if ((ch_line && _move_fw) || _set_f)
    {
        unsigned long backupStart = millis();

        while (abs(encoder1.Poss_L()) < bw_to_rotate)
        {
            robot.motor('A', -BACKUP_SPEED);
            robot.motor('C', -BACKUP_SPEED);

            if (millis() - backupStart > 2000UL)
            {
                Serial.println("TURN BACKUP FW TIMEOUT");
                break;
            }

            delay(1);
        }

        robot.motor('A', 15);
        robot.motor('C', 15);
        delay(25);
    }
    else if ((ch_line && _move_bw) || _set_b)
    {
        unsigned long backupStart = millis();

        while (abs(encoder1.Poss_L()) < fw_to_rotate)
        {
            robot.motor('A', BACKUP_SPEED);
            robot.motor('C', BACKUP_SPEED);

            if (millis() - backupStart > 2000UL)
            {
                Serial.println("TURN BACKUP BW TIMEOUT");
                break;
            }

            delay(1);
        }

        robot.motor('A', -15);
        robot.motor('C', -15);
        delay(20);
    }

    robot.motor('A', 0);
    robot.motor('C', 0);

    delay(100);

    // ========================================================
    // TARGET
    // ========================================================

    imu.update();

    const float startAngle =
        imu.yaw();

    float targetAngle;

    if (absolute)
    {
        targetAngle =
            constrainAngle180(degree);
    }
    else
    {
        targetAngle =
            constrainAngle180(
                startAngle + degree
            );
    }

    degree_turnGyro =
        (int)roundf(targetAngle);

    float error =
        constrainAngle180(
            targetAngle - startAngle
        );

    if (fabs(error) <= STOP_ERROR)
    {
        robot.motor('A', 0);
        robot.motor('C', 0);

        Serial.println("TURN ALREADY AT TARGET");
        return;
    }

    // ========================================================
    // PID VARIABLES
    // ========================================================

    float integral = 0.0f;
    float lastError = error;
    float dFilter = 0.0f;

    uint8_t stableCount = 0;
    uint8_t reverseCount = 0;

    unsigned long lastTime =
        micros();

    unsigned long startTime =
        millis();

    unsigned long timeout =
        1000UL +
        (unsigned long)(
            fabs(error) * 25.0f
        );

    timeout =
        constrain(
            timeout,
            1000UL,
            3000UL
        );

    float lastMoveAngle =
        startAngle;

    unsigned long lastMoveTime =
        millis();

    // ========================================================
    // MAIN LOOP
    // ========================================================

    while (true)
    {
        imu.update();

        const float currentAngle =
            imu.yaw();

        error =
            constrainAngle180(
                targetAngle -
                currentAngle
            );

        const float absError =
            fabs(error);

        // ====================================================
        // STOP ZONE
        // ====================================================

        if (absError <= STOP_ERROR)
        {
            robot.motor('A', 0);
            robot.motor('C', 0);

            stableCount++;

            if (stableCount >= STOP_STABLE_COUNT)
            {
                Serial.println("TURN TARGET STABLE");
                break;
            }

            delay(6);
            continue;
        }

        stableCount = 0;

        // ====================================================
        // DELTA TIME
        // ====================================================

        const unsigned long nowMicros =
            micros();

        float dt =
            (nowMicros - lastTime) *
            0.000001f;

        lastTime =
            nowMicros;

        dt =
            constrain(
                dt,
                0.003f,
                0.05f
            );

        const bool turningRight =
            error > 0.0f;

        // ====================================================
        // TARGET CROSSING
        // ====================================================

        const bool crossedTarget =
            (
                error > 0.0f &&
                lastError < 0.0f
            ) ||
            (
                error < 0.0f &&
                lastError > 0.0f
            );

        if (crossedTarget)
        {
            integral = 0.0f;
            dFilter = 0.0f;

            reverseCount++;

            // หยุดก่อนเปลี่ยนทิศ ลดการเด้ง
            robot.motor('A', 0);
            robot.motor('C', 0);

            delay(35);

            Serial.print("TURN CROSSED TARGET | Count=");
            Serial.print(reverseCount);

            Serial.print(" | Error=");
            Serial.println(error, 2);

            // หากกลับทิศหลายครั้ง ให้ยอมจบ
            if (reverseCount >= 3 && absError <= 4.0f)
            {
                Serial.println(
                    "TURN STOP AFTER MULTIPLE CORRECTIONS"
                );

                break;
            }
        }

        // ====================================================
        // CORRECTION ZONE
        // ใช้กำลังต่ำคงที่ ไม่ใช้ PID เต็ม
        // ====================================================

        if (absError <= CORRECTION_ZONE)
        {
            int correctionPower =
                CORRECTION_POWER;

            // Error มากขึ้น เพิ่มกำลังเล็กน้อย
            correctionPower +=
                (int)((absError - STOP_ERROR) * 1.2f);

            correctionPower =
                constrain(
                    correctionPower,
                    CORRECTION_POWER,
                    CORRECTION_MAX_POWER
                );

            int leftMotor;
            int rightMotor;

            if (turningRight)
            {
                leftMotor = correctionPower;
                rightMotor = -correctionPower;
            }
            else
            {
                leftMotor = -correctionPower;
                rightMotor = correctionPower;
            }

            robot.motor('A', leftMotor);
            robot.motor('C', rightMotor);

            lastError = error;

            // Timeout ยังต้องตรวจในเขตนี้
            if (millis() - startTime >= timeout)
            {
                Serial.print(
                    "TURN TIMEOUT IN CORRECTION | Error="
                );

                Serial.println(error, 2);
                break;
            }

            delay(6);
            continue;
        }

        // ====================================================
        // PID เฉพาะตอนห่างจากเป้าหมายมากกว่า 6°
        // ====================================================

        if (absError < 20.0f)
        {
            integral += error * dt;

            integral =
                constrain(
                    integral,
                    -10.0f,
                    10.0f
                );
        }
        else
        {
            integral = 0.0f;
        }

        const float derivative =
            (error - lastError) /
            dt;

        dFilter =
            KD_FILTER * dFilter +
            (1.0f - KD_FILTER) *
            derivative;

        lastError =
            error;

        const float kp =
            turningRight ? r_kp : l_kp;

        const float ki =
            turningRight ? r_ki : l_ki;

        const float kd =
            turningRight ? r_kd : l_kd;

        float pidOutput =
            kp * error +
            ki * integral +
            kd * 0.20f * dFilter;

        // ====================================================
        // SPEED PROFILE
        // ====================================================

        float maxOut;

        if (absError > 45.0f)
        {
            maxOut =
                maxSpeed;
        }
        else if (absError > 20.0f)
        {
            maxOut =
                maxSpeed * 0.72f;
        }
        else
        {
            maxOut =
                maxSpeed * 0.52f;
        }

        maxOut =
            max(
                maxOut,
                (float)MIN_POWER_NORMAL
            );

        // ใช้เฉพาะขนาด PID
        float outputMagnitude =
            fabs(pidOutput);

        outputMagnitude =
            constrain(
                outputMagnitude,
                (float)MIN_POWER_NORMAL,
                maxOut
            );

        float output =
            turningRight
                ? outputMagnitude
                : -outputMagnitude;

        // ====================================================
        // MOTOR BIAS
        // ====================================================

        const float leftBias =
            turningRight
                ? turnR_left_bias
                : turnL_left_bias;

        const float rightBias =
            turningRight
                ? turnR_right_bias
                : turnL_right_bias;

        int leftMotor =
            (int)roundf(
                output *
                leftBias
            );

        int rightMotor =
            (int)roundf(
                -output *
                rightBias
            );

        leftMotor =
            constrain(
                leftMotor,
                -100,
                100
            );

        rightMotor =
            constrain(
                rightMotor,
                -100,
                100
            );

        robot.motor('A', leftMotor);
        robot.motor('C', rightMotor);

        // ====================================================
        // STUCK DETECTION
        // ====================================================

        const float movedAngle =
            fabs(
                constrainAngle180(
                    currentAngle -
                    lastMoveAngle
                )
            );

        if (
            movedAngle >=
            MOVE_DETECTION_ANGLE
        )
        {
            lastMoveAngle =
                currentAngle;

            lastMoveTime =
                millis();
        }
        else if (
            millis() - lastMoveTime >=
            STUCK_TIME
        )
        {
            /*
               ถ้าหยุดหมุน ให้กระตุกเพิ่มเล็กน้อย
               แต่ไม่เพิ่มกำลังสะสมจนแรงเกินไป
            */

            int kickPower =
                MIN_POWER_NORMAL + 5;

            if (turningRight)
            {
                robot.motor('A', kickPower);
                robot.motor('C', -kickPower);
            }
            else
            {
                robot.motor('A', -kickPower);
                robot.motor('C', kickPower);
            }

            delay(45);

            lastMoveAngle =
                currentAngle;

            lastMoveTime =
                millis();

            Serial.println("TURN SMALL KICK");
        }

        // ====================================================
        // TIMEOUT
        // ====================================================

        if (
            millis() - startTime >=
            timeout
        )
        {
            robot.motor('A', 0);
            robot.motor('C', 0);

            Serial.print("TURN TIMEOUT | Error=");
            Serial.print(error, 2);

            Serial.print(" | Motor=");
            Serial.print(leftMotor);

            Serial.print(",");
            Serial.println(rightMotor);

            break;
        }

        delay(4);
    }

    // ========================================================
    // FINAL STOP
    // ========================================================

    robot.motor('A', 0);
    robot.motor('C', 0);

    delay(80);

    imu.update();

    const float finalAngle =
        imu.yaw();

    const float finalError =
        constrainAngle180(
            targetAngle -
            finalAngle
        );

    Serial.print("START : ");
    Serial.print(startAngle, 2);

    Serial.print(" TARGET : ");
    Serial.print(targetAngle, 2);

    Serial.print(" FINAL : ");
    Serial.print(finalAngle, 2);

    Serial.print(" ERROR : ");
    Serial.println(finalError, 2);
}

#endif  // TURNGYROPID_H