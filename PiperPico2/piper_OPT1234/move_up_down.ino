// =============================
// ARM CONTROL - Competition Ready Version (Fixed Down Overshoot)
// =============================
#include "MoveGyroPID.h"

float PULSES_PER_CM = 210.00f;

// Overshoot Compensation (ปรับใหม่ตามผลทดสอบ)
const float UP_OVERSHOOT_CM   = 1.10f;   // เพิ่มจาก 0.30 → 1.10 (แก้ปัญหาขึ้นเกิน)
const float DOWN_OVERSHOOT_CM = 0.35f;   // ใช้ค่าที่ลงมาแม่นยำแล้ว

static float current_position_cm = 0.0f;

// ==================== Global PID สำหรับเดิน ====================
float move_kp = 1.45f;
float move_ki = 0.015f;
float move_kd = 0.042f;

float last_PULSES_up = 0.0;


void set_PULSES_updown( float cm)
    {
        PULSES_PER_CM = cm;
    }
void set_move_pid(float kp, float ki, float kd) {
  move_kp = kp;
  move_ki = ki;
  move_kd = kd;
}

// ==================== Global สำหรับแขน ====================
bool arm_moving = false;
bool arm_homing_mode = false;
int  arm_direction = 0;
long arm_goal_pulses = 0;
unsigned long arm_last_time = 0;
float last_arm_command = -999.0f;

// State สำหรับ Homing (เหมือน arm_home_simple())
enum ArmHomingState {
  HOMING_DOWN,
  HOMING_BACKUP,
  HOMING_SLOW_UP
};
ArmHomingState homing_state = HOMING_DOWN;
unsigned long homing_backup_start = 0;

// =============================
// ARM_UPDOWN Blocking
// =============================
void arm_updown(float target_cm)
{
    last_PULSES_up = encoder2.Poss_R();
    arm_updown_nonblocking(target_cm);

    unsigned long timeout = millis() + 12000;
    while (arm_moving && millis() < timeout) {
        arm_update_nonblocking();
        delay(5);
    }

    if (arm_moving) {
        Serial.println("ARM TIMEOUT!");
        robot.motor('D', 0);
        arm_moving = false;
        arm_homing_mode = false;
    }
}

// =============================
// ARM_UPDOWN_NONBLOCKING (ปรับใหม่)
// =============================
void arm_updown_nonblocking(float target_cm)
{
    if (fabs(target_cm - last_arm_command) < 0.08f) return;
    last_arm_command = target_cm;

    if (fabs(target_cm) < 0.1f)   // HOMING
    {
        arm_homing_mode = true;
        homing_state = HOMING_DOWN;
        arm_moving = true;
        robot.motor('D', -70);
        arm_last_time = millis();
        Serial.println("=== ARM HOMING STARTED ===");
        return;
    }

    arm_homing_mode = false;
    if (arm_moving) {
        Serial.println("Arm is busy, command ignored");
        return;
    }

    float delta_cm = target_cm - current_position_cm;
    bool going_up = (delta_cm > 0.0f);

    // Overshoot Compensation (สำคัญ!)
    if (going_up) {
        delta_cm -= UP_OVERSHOOT_CM;      // ชดเชยมากขึ้นตอนขึ้น
    } else {
        delta_cm += DOWN_OVERSHOOT_CM;
    }

    // ป้องกัน delta 太เล็ก
    if (fabs(delta_cm) < 0.25f) {
        delta_cm = going_up ? 0.25f : -0.25f;
    }

    long start_pulses = encoder2.Poss_R();
    long target_pulses = (long)(fabs(delta_cm) * PULSES_PER_CM + 0.5f);

    arm_goal_pulses = start_pulses + (going_up ? target_pulses : -target_pulses);
    arm_direction = going_up ? 1 : -1;
    arm_moving = true;

    int start_speed = going_up ? 48 : 58;
    robot.motor('D', arm_direction * start_speed);
    arm_last_time = millis();

    Serial.print("Arm Command -> ");
    Serial.print(target_cm, 1);
    Serial.print(" cm | delta=");
    Serial.print(delta_cm, 2);
    Serial.print(" | Dir=");
    Serial.println(going_up ? "UP" : "DOWN");
}

// =============================
// ARM_UPDATE_NONBLOCKING
// =============================

void arm_update_nonblocking()
{
    if (!arm_moving) return;

    int adc_value = robot.adcRead(6);

    if (arm_homing_mode)
    {
        switch (homing_state)
        {
            case HOMING_DOWN:
                if (adc_value > 1500)
                {
                    robot.motor('D', 50);
                    homing_backup_start = millis();
                    homing_state = HOMING_BACKUP;
                    Serial.println("Limit switch triggered → Backup");
                }
                else if (millis() - arm_last_time > 10)
                {
                    robot.motor('D', -50);
                    arm_last_time = millis();
                }
                break;

            case HOMING_BACKUP:
                if (millis() - homing_backup_start >= 25)
                {
                    robot.motor('D', 50);
                    homing_state = HOMING_SLOW_UP;
                }
                break;

            case HOMING_SLOW_UP:
                if (adc_value <= 1500)
                {
                    robot.motor('D', 0);
                    encoder2.resetEncoders();
                    current_position_cm = 0.0f;
                    arm_moving = false;
                    arm_homing_mode = false;
                    Serial.println("=== HOMING COMPLETED - Arm at 0 cm ===");
                }
                else if (millis() - arm_last_time > 8)
                {
                    robot.motor('D', 50);
                    arm_last_time = millis();
                }
                break;
        }
        return;
    }

    // Normal Move
    long current = encoder2.Poss_R();
    bool reached = false;

    if (arm_direction > 0) {                    // UP
        if (current >= arm_goal_pulses - 12) reached = true;
    } else {                                    // DOWN
        if (current <= arm_goal_pulses + 16) reached = true;
    }

    if (reached)
    {
        int brake_power = (arm_direction > 0) ? 28 : 48;
        robot.motor('D', -arm_direction * brake_power);
        delay(arm_direction > 0 ? 35 : 48);
        robot.motor('D', 0);

        current_position_cm = (float)encoder2.Poss_R() / PULSES_PER_CM;
        arm_moving = false;

        Serial.print("Arm ARRIVED at: ");
        Serial.print(current_position_cm, 2);
        Serial.println(" cm");
        return;
    }

    if (millis() - arm_last_time > 7) {
        robot.motor('D', arm_direction * 92);
        arm_last_time = millis();
    }
}
// =============================
// MOVE FUNCTION สำหรับแข่ง
// =============================
void move(int sl, int sr, float kp, float dist, float arm_target)
{
    extern PiperPico2 robot;
    extern my_BMI160 gyro;
    extern EncoderLibrarys encoder1;
    extern EncoderLibraryss encoder2;

    if (dist <= 0.1f) return;

    if (_set_b == true) {
        dist += 5;
    }

    encoder1.resetEncoders();

    // GYRO SETUP
    gyro.resetAngles();
    float targetAngle = 0.0f;
    for (int i = 0; i < 3; i++) {
        targetAngle += gyro.gyro('z');
        delay(2);
    }
    targetAngle /= 3.0f;

    // เริ่มแขน
    arm_updown_nonblocking(arm_target);

    // PID Variables
    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();

    int leftBase  = constrain(sl, -100, 100);
    int rightBase = constrain(sr, -100, 100);

    long startPulses = encoder1.Poss_L();
    bool movingBackward = (leftBase < 0 && rightBase < 0);
    float pulsesPerCm_used = movingBackward ? pulsesPerCm_bw : pulsesPerCm_fw;

    const int MIN_SPEED = 22;
    float maxSpeed = (abs(leftBase) + abs(rightBase)) / 2.0f;
    const float ACC_DIST = 15.0f;
    const float SLOW_DIST = 20.0f + maxSpeed * 0.40f;

    bool wasDecelerating = false;

    while (true)
    {
        float currentAngle = gyro.gyro('z');
        long currentPulses = encoder1.Poss_L();
        long pulses = labs(currentPulses - startPulses);

        float traveled = pulses / pulsesPerCm_used;
        float remain = dist - traveled;

        // อัปเดตแขนทุก loop
        arm_update_nonblocking();

        // STOP + BRAKE
        if (remain <= 9.0f)
        {
            if (remain > 4.0f) {
                robot.motor('A', (int)(leftBase * 0.48f));
                robot.motor('C', (int)(rightBase * 0.48f));
                delay(12);
            } else {
                float brakeFactor = 0.38f;
                robot.motor('A', (int)(-leftBase * brakeFactor));
                robot.motor('C', (int)(-rightBase * brakeFactor));
                delay(14);
                robot.motor('A', 0);
                robot.motor('C', 0);
                break;
            }
        }

        // SPEED PROFILE + PID (เหมือนเดิม)
        int baseLeft  = leftBase;
        int baseRight = rightBase;

        if (traveled < ACC_DIST) {
            float p = traveled / ACC_DIST;
            baseLeft  = (int)(MIN_SPEED + (abs(leftBase)  - MIN_SPEED) * p) * (leftBase  >= 0 ? 1 : -1);
            baseRight = (int)(MIN_SPEED + (abs(rightBase) - MIN_SPEED) * p) * (rightBase >= 0 ? 1 : -1);
        }
        else if (remain < SLOW_DIST) {
            float p = remain / SLOW_DIST;
            p = constrain(p, 0.0f, 1.0f);
            float slowFactor = 0.78f + 0.22f * p;
            baseLeft  = (int)(MIN_SPEED + (abs(leftBase)  - MIN_SPEED) * slowFactor * p) * (leftBase  >= 0 ? 1 : -1);
            baseRight = (int)(MIN_SPEED + (abs(rightBase) - MIN_SPEED) * slowFactor * p) * (rightBase >= 0 ? 1 : -1);
        }

        float error = constrainAngle180(targetAngle - currentAngle);
        if (movingBackward) error = -error;

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        if (dt < 0.002f) dt = 0.02f;

        bool isDecelerating = (traveled >= ACC_DIST && remain < SLOW_DIST);
        if (isDecelerating && !wasDecelerating) integral = 0.0f;
        wasDecelerating = isDecelerating;

        integral += error * dt;
        integral = constrain(integral, -65.0f, 65.0f);

        float derivative = (error - lastError) / dt;
        lastError = error;

        float correction = kp * error + move_ki * integral + move_kd * derivative;
        float speedFactor = constrain((abs(baseLeft) + abs(baseRight)) / 80.0f, 0.33f, 1.0f);
        correction *= speedFactor;
        correction = constrain(correction, -30.0f, 30.0f);

        int left  = baseLeft  + (movingBackward ? -correction : correction);
        int right = baseRight - (movingBackward ? -correction : correction);

        left  = constrain(left,  -100, 100);
        right = constrain(right, -100, 100);

        robot.motor('A', left);
        robot.motor('C', right);

        delay(4);
    }

    // จบ
    robot.motor('A', 0);
    robot.motor('C', 0);
    robot.motor('D', 0);
    arm_moving = false;
    arm_homing_mode = false;
    _set_f = false;
    _set_b = false;
}