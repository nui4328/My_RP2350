void slow2_hand() {
  for (int i = 80; i >= 20; i--) {
    arm_hand('L', i);
    arm_hand('R', i);
    delay(5);
  }
}

void servo_begin() {
  arm_shoulder('L', 70);  // ค่าเดิมคือ65
  arm_shoulder('R', 70);

  arm_hand('L', 50);
  arm_hand('R', 50);

  //arm_can('L', 30);  // 140
  //arm_can('R', 30);
}

void set_servo_begin() {
  arm_shoulder('L', 170);  // ค่าเดิมคือ65
  arm_shoulder('R', 170);

  arm_hand('L', 50);
  arm_hand('R', 50);

  arm_can('L', 30);  // 140
  arm_can('R', 30);
}
void move_mision_5cm() {
  arm_updown(0);
  arm_hand('L', 30);  // ฝ่ามือซ้ายหุบเข้า
  arm_hand('R', 30);
  arm_shoulder('L', 70);
  arm_shoulder('R', 70);
  delay(200);
  move(30, 30, 2.4, 30, 5);
}
void trim_servo_0(int trim) {
  trim_0 = trim;
}
void trim_servo_1(int trim) {
  trim_1 = trim;
}
void trim_servo_2(int trim) {
  trim_2 = trim;
}
void trim_servo_3(int trim) {
  trim_3 = trim;
}
void trim_servo_4(int trim) {
  trim_4 = trim;
}
void trim_servo_5(int trim) {
  trim_5 = trim;
}

void arm_can(char LR, int deg) {
  if (LR == 'L') {
    robot.setservo(4, deg + trim_4);
  } else if (LR == 'R') {
    robot.setservo(5, (180 - deg) + trim_5);
  }
}
void arm_hand(char LR, int deg) {
  if (LR == 'L') {
    robot.setservo(0, (180 - deg) + trim_0);
  } else if (LR == 'R') {
    robot.setservo(3, (180 - deg) + trim_3);
  }
}

void arm_shoulder(char LR, int deg) {
  if (LR == 'L' || LR == 'l') {
    robot.setservo(1, (180 - deg) - trim_1);
  } else if (LR == 'R' || LR == 'r') {
    robot.setservo(2, deg + trim_2);
  }
}


void slow_2hand() {
  for (int i = 80; i >= 20; i--) {
    arm_hand('L', i);
    arm_hand('R', i);
    delay(5);
  }
}

void arm_bridge()  ////////////////////// มือและแขนหุบเข้าเพื่อข้ามสะพาน
{
  arm_hand('L', 40);
  arm_hand('R', 40);
  arm_shoulder('L', 10);
  arm_shoulder('R', 10);

}
void arm_bridge_box()  ////////////////////// มือและแขนหุบเข้าเพื่อข้ามสะพาน
{
  arm_hand('L', 40);
  arm_hand('R', 40);
  arm_shoulder('L', arm_ready_L -15);
  arm_shoulder('R', arm_ready_R -15);

}

// =============================
// ARM CONTROL (Stable Version)
// =============================
/*
#include "MoveGyroPID.h"
const float PULSES_PER_CM = 197.3;   // ต้อง calibrate ตามจริง
const float OVERSHOOT_COMPENSATION_CM = 0.35; // ชดเชยตอนยกขึ้น

static float current_position_cm = 0.0f;

// ==================== ตัวแปร Global PID (สำหรับแข่ง) ====================
float move_kp = 1.45f;
float move_ki = 0.015f;
float move_kd = 0.042f;

// ==================== set_move_pid ====================
void set_move_pid(float kp, float ki, float kd) {
  move_kp = kp;
  move_ki = ki;
  move_kd = kd;
}

// ==================== ตัวแปร Global สำหรับแขน ====================
bool arm_moving = false;
bool arm_homing_mode = false;
int  arm_direction = 0;
long arm_goal_pulses = 0;
unsigned long arm_last_time = 0;
float last_arm_command = -999.0f;     // เก็บตำแหน่งคำสั่งล่าสุด


// =============================
// MOVE ARM
// =============================
void arm_updown(float target_cm)
{
  if (abs(target_cm) < 0.1f)
  {
    arm_home_simple();
    current_position_cm = 0;
    return;
  }

  long start_pulses = encoder2.Poss_R();

  float delta_cm = target_cm - current_position_cm;

  if (delta_cm > 0)
  {
    delta_cm -= OVERSHOOT_COMPENSATION_CM;
    if (delta_cm < 0) delta_cm = 0;
  }

  long target_pulses = delta_cm * PULSES_PER_CM;

  int direction = (delta_cm >= 0) ? 1 : -1;

  long goal = start_pulses + target_pulses;

  const int MAX_SPEED = 100;
  const int MIN_SPEED = 80;

  const float ACC_DIST = 2.0;
  const float SLOW_DIST = 2.5;

  robot.motor('D', direction * MIN_SPEED);

  while (true)
  {
    long pos = encoder2.Poss_R();
    long diff = pos - start_pulses;

    float moved_cm = abs(diff) / PULSES_PER_CM;
    float remain_cm = abs(delta_cm) - moved_cm;

    if (remain_cm <= 0)
      break;

    int speed = MAX_SPEED;

    // acceleration
    if (moved_cm < ACC_DIST)
    {
      float p = moved_cm / ACC_DIST;
      p = constrain(p,0,1);

      speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED)*p;
    }

    // deceleration
    if (remain_cm < SLOW_DIST)
    {
      float p = remain_cm / SLOW_DIST;
      p = constrain(p,0,1);

      speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED)*p;
    }

    robot.motor('D', direction * speed);

    delay(2);
  }

  // soft brake
  robot.motor('D', -direction * 25);
  delay(30);
  robot.motor('D', 0);

  // ===== อัปเดตตำแหน่งจริงจาก encoder =====
  long finalPulse = encoder2.Poss_R();
  current_position_cm = finalPulse / PULSES_PER_CM;
  Serial.print(encoder2.Poss_R());
}


// =============================
// HOME ARM
// =============================
void arm_home_simple()
{
  robot.motor('D', -90);

  while (robot.adcRead(6) < 1500)
  {
    delay(2);
  }

  robot.motor('D', 60);
  delay(25);

  robot.motor('D', 40);

  while (robot.adcRead(6) > 1500)
  {
    delay(2);
  }

  robot.motor('D', 0);

  encoder2.resetEncoders();

  current_position_cm = 0;
}


// ==================== ARM Smart Control (ไม่ขยับเมื่อตัวเลขเดิม) ====================
void arm_updown_nonblocking(float target_cm)
{
  // ==================== ป้องกันการขยับซ้ำ ====================
  if (fabs(target_cm - last_arm_command) < 0.08f) {
    return;   // ตำแหน่งเดียวกัน → ไม่ต้องขยับแขน
  }

  last_arm_command = target_cm;   // อัปเดตตำแหน่งคำสั่งล่าสุด

  // ==================== โหมด Homing (ลงถึง limit switch) ====================
  if (fabs(target_cm) < 0.1f)
  {
    arm_homing_mode = true;
    robot.motor('D', -90);
    arm_moving = true;
    arm_last_time = millis();
    Serial.println("Starting Arm Homing to 0...");
    return;
  }

  // ==================== โหมดยกแขนปกติ ====================
  arm_homing_mode = false;
  if (arm_moving) return;

  float delta_cm = target_cm - current_position_cm;
  bool going_up = (delta_cm > 0);

  if (going_up) {
    delta_cm -= 0.4f;
    if (delta_cm < 0) delta_cm = 0;
  }

  long start_pulses = encoder2.Poss_R();
  long target_pulses = (long)(fabs(delta_cm) * PULSES_PER_CM);

  arm_goal_pulses = start_pulses + (going_up ? target_pulses : -target_pulses);
  arm_direction = going_up ? 1 : -1;
  arm_moving = true;

  robot.motor('D', arm_direction * 45);
  arm_last_time = millis();
}

void arm_update_nonblocking()
{
  if (!arm_moving) return;

  // โหมด Homing
  if (arm_homing_mode)
  {
    if (robot.adcRead(6) > 1500)
    {
      robot.motor('D', 60);
      delay(25);
      robot.motor('D', 40);

      while (robot.adcRead(6) > 1500) delay(2);

      robot.motor('D', 0);
      encoder2.resetEncoders();
      current_position_cm = 0.0f;
      arm_moving = false;
      arm_homing_mode = false;
      Serial.println("Homing completed - Arm at 0");
      return;
    }

    if (millis() - arm_last_time > 8) {
      robot.motor('D', -92);
      arm_last_time = millis();
    }
    return;
  }

  // โหมดยกแขนปกติ
  long current = encoder2.Poss_R();
  bool reached = false;

  if (arm_direction > 0) {
    if (current >= arm_goal_pulses - 12) reached = true;
  } else {
    if (current <= arm_goal_pulses + 12) reached = true;
  }

  if (reached)
  {
    robot.motor('D', -arm_direction * 28);
    delay(30);
    robot.motor('D', 0);
    current_position_cm = (float)encoder2.Poss_R() / PULSES_PER_CM;
    arm_moving = false;
    Serial.println("Arm at: " + String(current_position_cm));
    return;
  }

  if (millis() - arm_last_time > 6) {
    robot.motor('D', arm_direction * 92);
    arm_last_time = millis();
  }
}

void move(int sl, int sr, float kp, float dist, float arm_target)
{
  extern PiperPico2 robot;
  extern my_BMI160 gyro;
  extern EncoderLibrarys encoder1;
  extern EncoderLibraryss encoder2;

  if(_set_b == true)
    {
      dist += 5; 
    }

  if (dist <= 0.1f) return;

  encoder1.resetEncoders();

  int leftBase  = constrain(sl, -100, 100);
  int rightBase = constrain(sr, -100, 100);

  long startPulses = encoder1.Poss_L();

  // ---------- GYRO SETUP ----------
  gyro.resetAngles();
  float targetAngle = 0.0f;
  for (int i = 0; i < 3; i++) {
    targetAngle += gyro.gyro('z');
    delay(2);
  }
  targetAngle /= 3.0f;

  // ========== เริ่มควบคุมแขน (Smart Version) ==========
  arm_updown_nonblocking(arm_target);

  // ---------- PID Variables ----------
  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  // ---------- MOTION PROFILE (ปรับใหม่) ----------
  const int MIN_SPEED = 22;
  float maxSpeed = (abs(leftBase) + abs(rightBase)) / 2.0f;
  
  const float ACC_DIST = 15.0f;
  const float SLOW_DIST = 20.0f + maxSpeed * 0.40f;   // ลดลงชัดเจนจากเดิม (เดิม ~62 cm)

  bool wasDecelerating = false;

  while (true)
  {
    float currentAngle = gyro.gyro('z');
    long currentPulses = encoder1.Poss_L();
    long pulses = labs(currentPulses - startPulses);

    bool movingBackward = (leftBase < 0 && rightBase < 0);
    float pulsesPerCm_used = movingBackward ? pulsesPerCm_bw : pulsesPerCm_fw;

    float traveled = pulses / pulsesPerCm_used;
    float remain = dist - traveled;

    // อัปเดตแขนทุก loop
    arm_update_nonblocking();

    // ---------- STOP CONDITION + BRAKE 2 STAGE ----------
    if (remain <= 9.0f) 
    {
      // Stage 1: Coast (ลดความเร็วเบา ๆ)
      if (remain > 4.0f) {
        int coastLeft  = (int)(leftBase  * 0.48f);
        int coastRight = (int)(rightBase * 0.48f);
        robot.motor('A', coastLeft);
        robot.motor('C', coastRight);
        delay(12);
      }
      // Stage 2: Brake แรงเมื่อใกล้มาก
      else {
        float brakeFactor = 0.38f;        // เพิ่มจาก 0.16 เป็น 0.38
        int brakeLeft  = (int)(-leftBase  * brakeFactor);
        int brakeRight = (int)(-rightBase * brakeFactor);

        robot.motor('A', brakeLeft);
        robot.motor('C', brakeRight);
        delay(14);                        // เพิ่ม delay เล็กน้อย
        robot.motor('A', 0);
        robot.motor('C', 0);
        break;
      }
    }

    // ---------- SPEED PROFILE ----------
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
      float slowFactor = 0.78f + 0.22f * p;     // ลดช้าลง (เดิมลดแรงเกิน)
      baseLeft  = (int)(MIN_SPEED + (abs(leftBase)  - MIN_SPEED) * slowFactor * p) * (leftBase  >= 0 ? 1 : -1);
      baseRight = (int)(MIN_SPEED + (abs(rightBase) - MIN_SPEED) * slowFactor * p) * (rightBase >= 0 ? 1 : -1);
    }

    // ========== PID CONTROL ==========
    float error = constrainAngle180(targetAngle - currentAngle);
    if (movingBackward) error = -error;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt < 0.002f) dt = 0.02f;

    bool isDecelerating = (traveled >= ACC_DIST && remain < SLOW_DIST);
    if (isDecelerating && !wasDecelerating) {
      integral = 0.0f;        // Reset integral ตอนเริ่ม decelerate
    }
    wasDecelerating = isDecelerating;

    integral += error * dt;
    integral = constrain(integral, -65.0f, 65.0f);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float correction = kp * error + move_ki * integral + move_kd * derivative;

    float currentBase = (abs(baseLeft) + abs(baseRight)) / 2.0f;
    float speedFactor = constrain(currentBase / 80.0f, 0.33f, 1.0f);
    correction *= speedFactor;
    correction = constrain(correction, -30.0f, 30.0f);   // ขยายขอบเขตเล็กน้อย

    // ---------- MOTOR OUTPUT ----------
    int left  = baseLeft  + correction;
    int right = baseRight - correction;

    if (movingBackward) {
      left  = baseLeft  - correction;
      right = baseRight + correction;
    }

    left  = constrain(left,  -100, 100);
    right = constrain(right, -100, 100);

    robot.motor('A', left);
    robot.motor('C', right);

    delay(4);
  }

  // จบการวิ่ง
  robot.motor('A', 0);
  robot.motor('C', 0);
  robot.motor('D', 0);
  arm_moving = false;
  arm_homing_mode = false;
  _set_b = false;
  
}

*/