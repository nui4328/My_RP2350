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
int pulsesPerRevolution_FW  = 160;    ////////////////////////////////////////////////////////>>>>ตั้งค่าระยะทาง
int pulsesPerRevolution_BW  = 160;

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

int last_target_cm;
void imu_display();
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

////////////////////////////////////////////////////////////////////////////////////////////////ฟังก์ชันหมุน
////////////////////////////////////////////////////////////////////////////////////////////////ฟังก์ชันหมุน
////////////////////////////////////////////////////////////////////////////////////////////////ฟังก์ชันหมุน
void turnGyro(int speed, float degree, bool absolute = false)
{
    // ========================================================
    // CONFIG
    // ========================================================

    const int maxSpeed =
        constrain(abs(speed), 40, 95);

    // ---------------- TARGET ----------------

    const float STOP_ERROR = 1.8f;
    const float STABLE_ERROR = 2.2f;

    // ต้องอยู่ในช่วงนี้ติดต่อกัน
    const uint8_t STOP_STABLE_COUNT = 6;

    // ---------------- SPEED ----------------

    const int MIN_POWER_NORMAL = 22;

    // กำลังขั้นต่ำที่ใช้แก้ใกล้เป้า
    const int CORRECTION_MIN_POWER = 13;

    // กำลังสูงสุดช่วงใกล้เป้า
    const int CORRECTION_MAX_POWER = 20;

    // ---------------- BRAKE ----------------

    const float BRAKE_ZONE = 12.0f;

    // angular velocity ที่ถือว่าเกือบหยุด
    const float STOP_RATE = 3.0f;

    // ---------------- STUCK ----------------

    const float MOVE_DETECTION_ANGLE = 0.25f;

    const unsigned long STUCK_TIME = 300UL;

    const int KICK_POWER = 24;

    // ---------------- FILTER ----------------

    const float RATE_FILTER = 0.75f;

    // ---------------- INTEGRAL ----------------

    // ไม่ต้องให้ I มีอิทธิพลมาก
    const float I_LIMIT = 8.0f;

    // ========================================================
    // BACKUP
    // ========================================================

    encoder1.resetEncoders();

    if (_set_f)
    {
        unsigned long backupStart = millis();

        while (abs(encoder1.Poss_R()) < bw_to_rotate)
        {
            robot.motor('A', -25);
            robot.motor('C', -25);

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
    else if (_set_b)
    {
        unsigned long backupStart = millis();

        while (abs(encoder1.Poss_R()) < fw_to_rotate)
        {
            robot.motor('A', 25);
            robot.motor('C', 25);

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
    else
    {
        robot.motor('A', 0);
        robot.motor('C', 0);
    }

    robot.motor('A', 0);
    robot.motor('C', 0);

    delay(100);

    // ========================================================
    // TARGET
    // ========================================================

    imu.update();

    const float startAngle = imu.yaw();

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
    // PID
    // ========================================================

    float integral = 0.0f;

    float lastError = error;

    float filteredRate = 0.0f;

    unsigned long lastTime = micros();

    unsigned long startTime = millis();

    // ========================================================
    // TIMEOUT
    // ========================================================

    unsigned long timeout =
        1200UL +
        (unsigned long)(
            fabs(error) * 30.0f
        );

    timeout =
        constrain(
            timeout,
            1500UL,
            5000UL
        );

    // ========================================================
    // STUCK
    // ========================================================

    float lastMoveAngle =
        startAngle;

    unsigned long lastMoveTime =
        millis();

    // ========================================================
    // STABILITY
    // ========================================================

    uint8_t stableCount = 0;

    // ========================================================
    // CROSSING
    // ========================================================

    bool wasCrossed = false;

    uint8_t reverseCount = 0;

    // ========================================================
    // MAIN LOOP
    // ========================================================

    while (true)
    {
        // ----------------------------------------------------
        // IMU
        // ----------------------------------------------------

        imu.update();

        const float currentAngle =
            imu.yaw();

        // ----------------------------------------------------
        // ERROR
        // ----------------------------------------------------

        error =
            constrainAngle180(
                targetAngle -
                currentAngle
            );

        const float absError =
            fabs(error);

        // ----------------------------------------------------
        // DT
        // ----------------------------------------------------

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
                0.004f,
                0.05f
            );

        // ----------------------------------------------------
        // ANGULAR RATE
        //
        // error ลดลง = กำลังเข้าเป้า
        // error เพิ่ม = กำลังออกจากเป้า
        // ----------------------------------------------------

        const float rawRate =
            (error - lastError) / dt;

        filteredRate =
            RATE_FILTER * filteredRate +
            (1.0f - RATE_FILTER) * rawRate;

        // ----------------------------------------------------
        // TARGET CROSSING
        // ----------------------------------------------------

        const bool crossedTarget =
            (
                lastError > 0.0f &&
                error < 0.0f
            ) ||
            (
                lastError < 0.0f &&
                error > 0.0f
            );

        if (crossedTarget)
        {
            reverseCount++;

            integral = 0.0f;

            wasCrossed = true;

            // หยุดทันทีเพื่อเบรก
            robot.motor('A', 0);
            robot.motor('C', 0);

            delay(20);

            // อ่านตำแหน่งใหม่หลังหยุด
            imu.update();

            float newAngle =
                imu.yaw();

            error =
                constrainAngle180(
                    targetAngle -
                    newAngle
                );

            lastError = error;

            filteredRate = 0.0f;

            Serial.print("TARGET CROSS | ");
            Serial.print(error, 2);

            Serial.print(" | Count=");
            Serial.println(reverseCount);

            // ถ้า crossing หลายครั้ง
            // และเข้าใกล้เป้ามากแล้ว ให้หยุด
            if (
                reverseCount >= 3 &&
                fabs(error) <= 3.0f
            )
            {
                Serial.println(
                    "TURN STOP MULTIPLE CROSS"
                );

                break;
            }
        }

        // ----------------------------------------------------
        // STABLE STOP
        //
        // ต้องทั้ง:
        // error เล็ก
        // และ rotation rate ต่ำ
        // ----------------------------------------------------

        if (
            absError <= STABLE_ERROR &&
            fabs(filteredRate) <= STOP_RATE
        )
        {
            robot.motor('A', 0);
            robot.motor('C', 0);

            stableCount++;

            lastError = error;

            if (
                stableCount >=
                STOP_STABLE_COUNT
            )
            {
                Serial.println(
                    "TURN TARGET STABLE"
                );

                break;
            }

            delay(6);

            continue;
        }

        stableCount = 0;

        // ----------------------------------------------------
        // PID GAIN
        // ----------------------------------------------------

        const bool turningRight =
            error > 0.0f;

        const float kp =
            turningRight
                ? r_kp
                : l_kp;

        const float ki =
            turningRight
                ? r_ki
                : l_ki;

        const float kd =
            turningRight
                ? r_kd
                : l_kd;

        // ----------------------------------------------------
        // INTEGRAL
        //
        // ใช้เฉพาะตอนใกล้เป้า
        // ----------------------------------------------------

        if (absError < 20.0f)
        {
            integral +=
                error * dt;

            integral =
                constrain(
                    integral,
                    -I_LIMIT,
                    I_LIMIT
                );
        }
        else
        {
            integral = 0.0f;
        }

        // ----------------------------------------------------
        // PD
        // ----------------------------------------------------

        float pidOutput =
            kp * error +
            ki * integral +
            kd * filteredRate;

        // ----------------------------------------------------
        // BRAKING
        //
        // ถ้าเข้าเป้าเร็วเกินไป
        // ลดกำลัง
        // ----------------------------------------------------

        float brakeFactor = 1.0f;

        if (absError < BRAKE_ZONE)
        {
            brakeFactor =
                absError /
                BRAKE_ZONE;

            brakeFactor =
                constrain(
                    brakeFactor,
                    0.20f,
                    1.0f
                );
        }

        // ----------------------------------------------------
        // OUTPUT
        // ----------------------------------------------------

        float output =
            pidOutput *
            brakeFactor;

        // ----------------------------------------------------
        // SPEED LIMIT
        // ----------------------------------------------------

        float maxOut;

        if (absError > 45.0f)
        {
            maxOut =
                maxSpeed;
        }
        else if (absError > 20.0f)
        {
            maxOut =
                maxSpeed * 0.78f;
        }
        else if (absError > 10.0f)
        {
            maxOut =
                maxSpeed * 0.58f;
        }
        else
        {
            maxOut =
                maxSpeed * 0.40f;
        }

        maxOut =
            max(
                maxOut,
                (float)MIN_POWER_NORMAL
            );

        output =
            constrain(
                output,
                -maxOut,
                maxOut
            );

        // ----------------------------------------------------
        // NEAR TARGET CORRECTION
        //
        // 2° - 10°
        // ----------------------------------------------------

        if (
            absError > STOP_ERROR &&
            absError <= 10.0f
        )
        {
            float correctionPower;

            correctionPower =
                CORRECTION_MIN_POWER +
                (
                    absError -
                    STOP_ERROR
                ) * 1.4f;

            correctionPower =
                constrain(
                    correctionPower,
                    (float)CORRECTION_MIN_POWER,
                    (float)CORRECTION_MAX_POWER
                );

            // ถ้ากำลังเข้าเป้าเร็ว
            // ลดแรงอีก
            if (fabs(filteredRate) > 20.0f)
            {
                correctionPower *= 0.70f;
            }

            correctionPower =
                constrain(
                    correctionPower,
                    11.0f,
                    (float)CORRECTION_MAX_POWER
                );

            output =
                turningRight
                    ? correctionPower
                    : -correctionPower;
        }

        // ----------------------------------------------------
        // MOTOR BIAS
        // ----------------------------------------------------

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

        robot.motor(
            'A',
            leftMotor
        );

        robot.motor(
            'C',
            rightMotor
        );

        // ----------------------------------------------------
        // STUCK DETECTION
        //
        // สำคัญ:
        // ทำงานแม้ใน correction zone
        // ----------------------------------------------------

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
            millis() -
            lastMoveTime >=
            STUCK_TIME
        )
        {
            // --------------------------------------------
            // KICK
            // --------------------------------------------

            int kick =
                KICK_POWER;

            // ถ้าใกล้เป้ามาก
            // ลด kick
            if (absError < 6.0f)
            {
                kick = 18;
            }

            if (turningRight)
            {
                robot.motor(
                    'A',
                    kick
                );

                robot.motor(
                    'C',
                    -kick
                );
            }
            else
            {
                robot.motor(
                    'A',
                    -kick
                );

                robot.motor(
                    'C',
                    kick
                );
            }

            delay(35);

            robot.motor('A', 0);
            robot.motor('C', 0);

            delay(8);

            imu.update();

            lastMoveAngle =
                imu.yaw();

            lastMoveTime =
                millis();

            Serial.println(
                "TURN SMALL KICK"
            );
        }

        // ----------------------------------------------------
        // TIMEOUT
        // ----------------------------------------------------

        if (
            millis() -
            startTime >=
            timeout
        )
        {
            robot.motor('A', 0);
            robot.motor('C', 0);

            Serial.print(
                "TURN TIMEOUT | Error="
            );

            Serial.print(
                error,
                2
            );

            Serial.print(
                " | Rate="
            );

            Serial.print(
                filteredRate,
                2
            );

            Serial.print(
                " | Motor="
            );

            Serial.print(
                leftMotor
            );

            Serial.print(",");

            Serial.println(
                rightMotor
            );

            break;
        }

        // ----------------------------------------------------
        // SAVE ERROR
        // ----------------------------------------------------

        lastError =
            error;

        delay(4);
    }

    // ========================================================
    // FINAL STOP
    // ========================================================

    robot.motor('A', 0);
    robot.motor('C', 0);

    delay(60);

    imu.update();

    const float finalAngle =
        imu.yaw();

    const float finalError =
        constrainAngle180(
            targetAngle -
            finalAngle
        );

    // ========================================================
    // DISPLAY
    // ========================================================

    imu_display();

    Serial.println();
    Serial.println(
        "========== TURN RESULT =========="
    );

    Serial.print(
        "START  : "
    );

    Serial.println(
        startAngle,
        2
    );

    Serial.print(
        "TARGET : "
    );

    Serial.println(
        targetAngle,
        2
    );

    Serial.print(
        "FINAL  : "
    );

    Serial.println(
        finalAngle,
        2
    );

    Serial.print(
        "ERROR  : "
    );

    Serial.println(
        finalError,
        2
    );

    Serial.println(
        "================================="
    );
}
#endif  // TURNGYROPID_H