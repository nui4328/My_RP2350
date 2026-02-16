void move_bw(int sl, float kp, float dist) {
  
  extern ArrayPico2 robot;
  extern EncoderLibrarys encoder;
  extern my_BMI160 gyro;

  encoder.resetEncoders();  

  float targetPulses = dist * pulsesPerCm;
  long startPulsesL = encoder.Poss_R();   // ใช้ encoder เดียวกัน แต่ค่าจะติดลบตอนถอย

  gyro.resetAngles();
  delay(30);
  float initialAngle = averageGyroZ(10);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  Serial.printf("=== move_bw | speed: %d | kp: %.2f | dist: %.1f cm ===\n", sl, kp, dist);

  const int side_threshold  = 30;   
  const int front_threshold = 50;   
  const int very_low_speed  = 12;     

  // Ramp up (ออกตัวช้า) เหมือน move_fw
  const float RAMP_UP_DISTANCE = 15.0f;     
  bool has_reached_full_speed = false;

  // Ramp down (ช้าลงตอนใกล้เป้า) เหมือนเดิม
  bool slowing = false;

  while (true) {
    delay(5);  

    long currentPulses = encoder.Poss_R();
    // เพราะถอยหลัง ค่าจะลดลง → ใช้ abs เพื่อคำนวณระยะที่เดินไป
    long pulsesTraveled = abs(currentPulses - startPulsesL);
    float distanceTraveled = pulsesTraveled / pulsesPerCm;
    float remaining = dist - distanceTraveled;

    // ==============================================
    // 1. Ramp up: ออกตัวช้า (เหมือน move_fw)
    // ==============================================
    int base_speed = sl;

    if (distanceTraveled < RAMP_UP_DISTANCE && !has_reached_full_speed) {
      float progress = distanceTraveled / RAMP_UP_DISTANCE;
      progress = constrain(progress, 0.0f, 1.0f);
      
      base_speed = very_low_speed + (int)((sl - very_low_speed) * progress);
      
      if (progress >= 0.98f) {
        has_reached_full_speed = true;
        Serial.println(">>> ออกตัวช้า (ถอย) เสร็จ → ความเร็วเต็มแล้ว");
      }
    }

    // ==============================================
    // 2. ช้าลงตอนใกล้เป้า (เหมือน move_fw)
    // ==============================================
    if (remaining <= dist * 0.60f) {  
      slowing = true;

      float fade = remaining / (dist * 0.50f);  
      fade = constrain(fade, 0.0f, 1.0f);

      base_speed = (int)(very_low_speed + (sl - very_low_speed) * fade * 0.25f);
      base_speed = max(base_speed, very_low_speed);

      static bool has_logged = false;
      if (!has_logged) {
        has_logged = true;
        Serial.printf(">>> เข้าช่วงช้าลง (ถอย) เหลือ %.1f cm → speed ~%d\n", 
                      remaining, base_speed);
      }
    }

    // หยุดเมื่อใกล้เป้ามาก + เบรกนุ่ม
    if (remaining <= 2.0f) {
      robot.Motor(10, 10);
      delay(10);
      robot.Motor(0, 0);
      
      Serial.println("ถึงเป้า (ถอยหลัง) แล้ว - เบรกนุ่มเรียบร้อย");
      break;
    }

    // ==============================================
    // PID gyro (ทิศทางเดียวกับ move_fw)
    // ==============================================
    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(targetAngle + currentAngle);  // ใช้สูตรเดียวกัน

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt <= 0.001f) dt = 0.02f;
    if (dt > 0.08f) dt = 0.02f;

    integral += angleError * dt;
    integral = constrain(integral, -60, 60);

    float derivative = (angleError - lastError) / dt;
    lastError = angleError;

    float correction = kp * angleError + 0.001f * integral + 0.02f * derivative;
    correction = constrain(correction, -20, 20);

    // ==============================================
    // สำคัญ: ความเร็วต้องติดลบ เพราะเป็นการถอยหลัง
    // ==============================================
    int leftSpeed  = -base_speed - correction;   // ลบเพื่อถอย
    int rightSpeed = -base_speed + correction;   // ลบเพื่อถอย (correction ยังคง + - เหมือนเดิม)

    leftSpeed  = constrain(leftSpeed, -100, -very_low_speed);
    rightSpeed = constrain(rightSpeed, -100, -very_low_speed);
    robot.Motor(leftSpeed, rightSpeed);
    // ==============================================
      
  }

  // เบรกนุ่มทิ้งท้าย
  robot.Motor(10, 10);
  delay(30);
  robot.Motor(0, 0);

  
}

void distance_bw(int bw)
  {
    bw_distance = bw;
  }
