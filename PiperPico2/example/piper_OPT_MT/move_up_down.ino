#include "MoveGyroPID.h"

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
#define MAX_SAFE_ARM_HEIGHT   15.0f
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

// =============================
// ARM_UPDOWN (Blocking)
// =============================
void arm_updown(float target_cm)
{
    target_cm = constrain(target_cm, 0.0f, 15.0f);

    last_PULSES_up = encoder2.Poss_R();
    arm_updown_nonblocking(target_cm);

    unsigned long timeout = millis() + 15000;
    while (arm_moving && millis() < timeout) {
        arm_update_nonblocking();
        delay(5);
    }

    if (arm_moving) {
        Serial.println("ARM TIMEOUT!");
        robot.motor('D', 0);
        arm_moving = false;
    }

    Serial.printf("🏁 arm_updown(%.1f) FINISHED → Current Position = %.2f cm\n", 
                  target_cm, current_position_cm);
}

// =============================
// ARM_UPDOWN_NONBLOCKING
// =============================
void arm_updown_nonblocking(float target_cm)
{
    target_cm = constrain(target_cm, 0.0f, 15.0f);
    if (fabs(target_cm - last_arm_command) < 0.08f) return;
    last_arm_command = target_cm;

    if (fabs(target_cm) < 0.2f) {   // HOMING
        arm_homing_mode = true;
        homing_state = HOMING_DOWN;
        arm_moving = true;
        robot.motor('D', -40);
        arm_last_time = millis();
        arm_last_move_time = millis();
        arm_last_pulses_stuck = encoder2.Poss_R();
        Serial.println("=== ARM HOMING STARTED (DOWN SLOW) ===");
        return;
    }

    arm_homing_mode = false;
    if (arm_moving) return;

    float delta_cm = target_cm - current_position_cm;
    bool going_up = (delta_cm > 0.0f);

    if (going_up) delta_cm -= UP_OVERSHOOT_CM;
    else          delta_cm += DOWN_OVERSHOOT_CM;

    if (fabs(delta_cm) < 0.2f) delta_cm = going_up ? 0.25f : -0.25f;

    long start_pulses = encoder2.Poss_R();
    long target_pulses = (long)(fabs(delta_cm) * PULSES_PER_CM + 0.5f);

    arm_goal_pulses = start_pulses + (going_up ? target_pulses : -target_pulses);
    arm_direction = going_up ? 1 : -1;
    arm_moving = true;

    // ความเร็วเริ่มต้น - ช้าลงเมื่อเลื่อนลง
    int start_speed = going_up ? 55 : 38;        // ← ลงช้าครึ่งหนึ่ง
    robot.motor('D', arm_direction * start_speed);

    arm_last_time = millis();
    arm_last_move_time = millis();
    arm_last_pulses_stuck = start_pulses;

    Serial.printf("Arm Command -> %.1f cm | Dir=%s (from %.2f)\n", 
                  target_cm, going_up ? "UP" : "DOWN", current_position_cm);
}

// =============================
// ARM_UPDATE_NONBLOCKING (ชะลอ + ช้าลงเมื่อเลื่อนลง)
// =============================
void arm_update_nonblocking()
{
    if (!arm_moving) return;

    int adc_value = robot.adcRead(6);
    long current_pulses = encoder2.Poss_R();
    current_position_cm = (float)current_pulses / PULSES_PER_CM;

    if (arm_homing_mode) {
        // Homing (ไม่เปลี่ยน)
        switch (homing_state) {
            case HOMING_DOWN:
                if (adc_value >= 1500) {
                    robot.motor('D', 0);
                    delay(40);
                    robot.motor('D', 28);
                    homing_backup_start = millis();
                    homing_state = HOMING_BACKUP;
                } else if (millis() - arm_last_time > 8) {
                    robot.motor('D', -40);
                    arm_last_time = millis();
                }
                break;

            case HOMING_BACKUP:
                if (adc_value < 1500) {
                    robot.motor('D', 0);
                    delay(30);
                    encoder2.resetEncoders();
                    current_position_cm = 0.0f;
                    arm_moving = false;
                    arm_homing_mode = false;
                    Serial.println("=== HOMING COMPLETED - Arm at 0 cm ===");
                } else if (millis() - arm_last_time > 15) {
                    robot.motor('D', 26);
                    arm_last_time = millis();
                }
                break;
        }
        return;
    }

    // ==================== NORMAL MOVE ====================
    long delta_pulses = abs(current_pulses - arm_last_pulses_stuck);
    if (delta_pulses < ARM_MIN_PULSES) {
        if (millis() - arm_last_move_time > ARM_STUCK_TIME_MS) {
            Serial.println("🚨 ARM STUCK!");
            robot.motor('D', 0);
            arm_moving = false;
            return;
        }
    } else {
        arm_last_move_time = millis();
        arm_last_pulses_stuck = current_pulses;
    }

    // คำนวณระยะเหลือ
    long remain_pulses = (arm_direction > 0) ? 
        (arm_goal_pulses - current_pulses) : (current_pulses - arm_goal_pulses);

    // === ความเร็ว (ช้าลงเมื่อเลื่อนลง) ===
    int power = (arm_direction > 0) ? 92 : 40;   // หน้าคือขึ้น หลังคือลง  ลง = ครึ่งหนึ่ง    
    
    // ชะลอเมื่อใกล้ถึง (แนะนำ)
    if (remain_pulses < 55) {                    // ใกล้มาก (< 2.1 mm)
        power = (arm_direction > 0) ? 15 : 10;
    }
    else if (remain_pulses < 120) {              // ใกล้ปานกลาง (< 4.8 mm)
        power = (arm_direction > 0) ? 25 : 15;
    }
    else if (remain_pulses < 180) {              // เริ่มชะลอ
        power = (arm_direction > 0) ? 32 :20;
    }

    // ถึงเป้า
    bool reached = false;
    if (arm_direction > 0) {
        if (current_pulses >= arm_goal_pulses - 5) reached = true;
    } else {
        if (current_pulses <= arm_goal_pulses + 12) reached = true;
    }

    if (reached) {
        int brake = (arm_direction > 0) ? 35 : 38;           // กำลังเบรก
        robot.motor('D', -arm_direction * brake);            // ส่งกำลังย้อนกลับ
        delay(arm_direction > 0 ? 38 : 38);                  // รอให้หยุด
        robot.motor('D', 0);                                 // หยุดมอเตอร์;

        current_position_cm = (float)encoder2.Poss_R() / PULSES_PER_CM;
        arm_moving = false;
        Serial.printf("✅ Arm ARRIVED at: %.2f cm\n", current_position_cm);
        return;
    }

    // ส่งกำลัง
    if (millis() - arm_last_time > 6) {
        robot.motor('D', arm_direction * power);
        arm_last_time = millis();
    }
}


// =============================
// MOVE FUNCTION
// =============================
void move(int sl, int sr, float kp, float dist, float arm_target)
{
    extern PiperPico2 robot;
    extern my_BMI160 gyro;
    extern EncoderLibrarys encoder1;
    // extern EncoderLibraryss encoder2;  // ไม่ใช้

    if (dist <= 0.1f) return;

    if (_set_b == true) dist += 5;

    encoder1.resetEncoders();

    gyro.resetAngles();
    float targetAngle = 0.0f;
    for (int i = 0; i < 5; i++) {           // อ่านเฉลี่ย 5 ครั้ง
        targetAngle += gyro.gyro('z');
        delay(2);
    }
    targetAngle /= 5.0f;

    arm_updown_nonblocking(arm_target);

    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();

    int leftBase  = constrain(sl, -100, 100);
    int rightBase = constrain(sr, -100, 100);

    long startPulses = encoder1.Poss_L();
    bool movingBackward = (leftBase < 0 && rightBase < 0);
    float pulsesPerCm_used = movingBackward ? pulsesPerCm_bw : pulsesPerCm_fw;

    const int MIN_SPEED = 22;
    const float ACC_DIST = 15.0f;
    const float SLOW_DIST = 22.0f + (abs(leftBase) + abs(rightBase)) * 0.18f;  // ปรับตามความเร็ว

    bool wasDecelerating = false;

    while (true)
    {
        float currentAngle = gyro.gyro('z');
        long currentPulses = encoder1.Poss_L();
        long pulses = labs(currentPulses - startPulses);
        float traveled = pulses / pulsesPerCm_used;
        float remain = dist - traveled;

        arm_update_nonblocking();

        // ================== หยุดเมื่อถึงระยะ ==================
        if (remain <= 9.0f)
        {
            if (remain > 4.0f) {
                robot.motor('A', (int)(leftBase * 0.48f));
                robot.motor('C', (int)(rightBase * 0.48f));
            } else {
                if (break_move == 1) {
                    robot.motor('A', (int)(-leftBase * 0.48f));   // เบรก
                    robot.motor('C', (int)(-rightBase * 0.48f));
                    delay(16);
                }
                robot.motor('A', 0);
                robot.motor('C', 0);
                break;
            }
        }

        // ================== Accel / Decel ==================
        int baseLeft  = leftBase;
        int baseRight = rightBase;

        if (traveled < ACC_DIST) {
            float p = traveled / ACC_DIST;
            baseLeft  = (int)(MIN_SPEED + (abs(leftBase)  - MIN_SPEED) * p) * (leftBase  >= 0 ? 1 : -1);
            baseRight = (int)(MIN_SPEED + (abs(rightBase) - MIN_SPEED) * p) * (rightBase >= 0 ? 1 : -1);
        }
        else if (remain < SLOW_DIST) {
            float p = remain / SLOW_DIST;
            float slowFactor = 0.40f + 0.60f * p * p;   // quadratic decelerate นุ่ม
            baseLeft  = (int)(leftBase  * slowFactor);
            baseRight = (int)(rightBase * slowFactor);
        }

        // ================== PID Gyro ==================
        float error = constrainAngle180(targetAngle - currentAngle);
        if (movingBackward) error = -error;

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        if (dt < 0.001f) dt = 0.01f;
        lastTime = now;

        bool isDecelerating = (traveled >= ACC_DIST && remain < SLOW_DIST);
        if (isDecelerating && !wasDecelerating) integral = 0.0f;
        wasDecelerating = isDecelerating;

        integral += error * dt;
        integral = constrain(integral, -62.0f, 62.0f);

        float derivative = (error - lastError) / dt;
        lastError = error;

        float correction = kp * error + move_ki * integral + move_kd * derivative;
        correction = constrain(correction, -33.0f, 33.0f);

        int left  = baseLeft  + (movingBackward ? -correction : correction);
        int right = baseRight - (movingBackward ? -correction : correction);

        left  = constrain(left,  -100, 100);
        right = constrain(right, -100, 100);

        // ================== Line Sensor Correction ==================
        if (sl > 0) {  // เดินหน้า
            if (robot.adcRead(8) < robot.adcMD(8) && robot.adcRead(1) > robot.adcMD(1)) {
                robot.motor('A', left * 0.65f);
                robot.motor('C', right * 1.45f);
                targetAngle = gyro.gyro('z');        // สำคัญ! ไม่ reset
            } 
            else if (robot.adcRead(8) > robot.adcMD(8) && robot.adcRead(1) < robot.adcMD(1)) {
                robot.motor('A', left * 1.45f);
                robot.motor('C', right * 0.65f);
                targetAngle = gyro.gyro('z');
            } else {
                robot.motor('A', left);
                robot.motor('C', right);
            }
        } else {  // ถอยหลัง
            if (robot.adcRead(9) < robot.adcMD(9) && robot.adcRead(0) > robot.adcMD(0)) {
                robot.motor('A', left * 0.65f);
                robot.motor('C', right * 1.45f);
                targetAngle = gyro.gyro('z');
            } 
            else if (robot.adcRead(9) > robot.adcMD(9) && robot.adcRead(0) < robot.adcMD(0)) {
                robot.motor('A', left * 1.45f);
                robot.motor('C', right * 0.65f);
                targetAngle = gyro.gyro('z');
            } else {
                robot.motor('A', left);
                robot.motor('C', right);
            }
        }

        delay(4);
    }

    // Final stop
    robot.motor('A', 0);
    robot.motor('C', 0);
    robot.motor('D', 0);

    arm_moving = false;
    arm_homing_mode = false;
    _set_f = false;
    _set_b = false;
    break_move = 1;

    Serial.printf("🏁 move() FINISHED | Dist: %.1f cm | Arm: %.2f cm\n", 
                  dist, current_position_cm);
}

void move_chopsticks(int sl, int sr, float kp, float dist, float arm_target)
{
    extern PiperPico2 robot;
    extern my_BMI160 gyro;
    extern EncoderLibrarys encoder1;
    extern EncoderLibraryss encoder2;

    if (dist <= 0.1f) return;

    if (_set_b == true) dist += 5;

    encoder1.resetEncoders();

    gyro.resetAngles();
    float targetAngle = 0.0f;
    for (int i = 0; i < 3; i++) {
        targetAngle += gyro.gyro('z');
        delay(2);
    }
    targetAngle /= 3.0f;

    arm_updown_nonblocking(arm_target);

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

        arm_update_nonblocking();

        if (remain <= 9.0f)
        {
            if (remain > 4.0f) {
                robot.motor('A', (int)(leftBase * 0.48f));
                robot.motor('C', (int)(rightBase * 0.48f));
                delay(12);
            } else {
                if(break_move == 1) {
                    float brakeFactor = 0.38f;
                    robot.motor('A', (int)(-leftBase * brakeFactor));
                    robot.motor('C', (int)(-rightBase * brakeFactor));
                    delay(14);
                }
                robot.motor('A', 0);
                robot.motor('C', 0);
                break;
            }
        }

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
        robot.motor('A', left); robot.motor('C', right); 
      

        delay(4);
    }

    robot.motor('A', 0);
    robot.motor('C', 0);
    robot.motor('D', 0);
    arm_moving = false;
    arm_homing_mode = false;
    _set_f = false;
    _set_b = false;
    break_move = 1;

    Serial.printf("🏁 move() FINISHED → Arm Position = %.2f cm\n", current_position_cm);

}