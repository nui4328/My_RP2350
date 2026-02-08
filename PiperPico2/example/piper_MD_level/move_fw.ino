void move_fw(int sl, float kp, float dist, String _line) {
  
  extern PiperPico2 robot;
  extern my_BMI160 gyro;
  extern EncoderLibrarys encoder1;
  extern EncoderLibraryss encoder2;

  char lr = ' ';  
  encoder1.resetEncoders();
  _move_fw = true;  
  _move_bw = false;  

  if (_set_b == true) {
    dist += 7;
    _set_b = false;
  }

  float targetPulses = dist * pulsesPerCm;
  long startPulsesL = encoder1.Poss_R();

  gyro.resetAngles();
  delay(30);
  float initialAngle = averageGyroZ(10);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  Serial.printf("=== move_fw | speed: %d | kp: %.2f | dist: %.1f cm ===\n", sl, kp, dist);

  const int side_threshold  = 30;   
  const int front_threshold = 50;   
  const int very_low_speed  = 15;     // ความเร็วต่ำสุดช่วงช้า (ปรับได้ 8-14)

  // ────────────────────────────────────────────────
  // เพิ่มส่วน ramp up (ออกตัวช้า) เท่านั้น
  // ────────────────────────────────────────────────
  const float RAMP_UP_DISTANCE = 15.0f;     // ระยะที่ใช้เร่งจากช้า → เต็ม (หน่วย cm) ปรับได้
  float current_target_speed = very_low_speed;  // เริ่มต้นจากความเร็วต่ำ
  bool has_reached_full_speed = false;

  bool slowing = false;

  while (true) {
    delay(5);  
    long currentPulses = encoder1.Poss_R();
    long pulsesTraveled = abs(currentPulses - startPulsesL);
    float distanceTraveled = pulsesTraveled / pulsesPerCm;

    float remaining = dist - distanceTraveled;

    // ==============================================
    // 1. Ramp up: ค่อย ๆ เร่งความเร็วในช่วงแรก
    // ==============================================
    int base_speed = sl;

    if (distanceTraveled < RAMP_UP_DISTANCE && !has_reached_full_speed) {
      // เร่งแบบ linear ตามระยะทาง
      float progress = distanceTraveled / RAMP_UP_DISTANCE;
      progress = constrain(progress, 0.0f, 1.0f);
      
      base_speed = very_low_speed + (int)((sl - very_low_speed) * progress);
      
      // ถ้าใกล้ถึงเต็มแล้ว ให้ล็อกไว้ที่เต็ม
      if (progress >= 0.98f) {
        has_reached_full_speed = true;
        Serial.println(">>> ออกตัวช้าเสร็จ → ความเร็วเต็มแล้ว");
      }
    }

    // ==============================================
    // 2. ช่วงช้าลงตอนใกล้เป้า (ส่วนเดิมของคุณ ยังคงไว้เหมือนเดิม)
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
        Serial.printf(">>> เข้าช่วงช้าลง (เหลือ %.1f cm → speed ลดเหลือ ~%d)\n", 
                      remaining, base_speed);
      }
    }

    // หยุดเมื่อใกล้เป้ามาก + เบรกนุ่ม (เหมือนเดิม)
    if (remaining <= 2.0f) {
      robot.motor('A', -2); robot.motor('C', -2);
      delay(10);
      robot.motor('A', 0); robot.motor('C', 0);
      
      break;
    }

    // ==============================================
    // ส่วน gyro + PID คำนวณเหมือนเดิมทุกประการ
    // ==============================================
    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(targetAngle + currentAngle);
    // ถ้ายังเอียง ลองเปลี่ยนเป็น targetAngle - currentAngle ได้ครับ

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

    int leftSpeed  = base_speed - correction;
    int rightSpeed = base_speed + correction;

    leftSpeed  = constrain(leftSpeed, -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);

    // ==============================================
    // ตรวจเซ็นเซอร์ + จัดการเส้นข้าง / ตะกียบ เหมือนเดิม 100%
    // ==============================================
    if (robot.adcRead(0) < robot.adcMD(0) - side_threshold && 
        robot.adcRead(3) > robot.adcMD(3)) {
      robot.motor('A', leftSpeed*1.5); robot.motor('C', rightSpeed/3);
      gyro.resetAngles();
    } 
    else if (robot.adcRead(0) > robot.adcMD(0) && 
             robot.adcRead(3) < robot.adcMD(3) - side_threshold) {
      robot.motor('A', leftSpeed/3); robot.motor('C', rightSpeed*1.5);      
      gyro.resetAngles();
    } 
    else if (robot.adcRead(1) < robot.adcMD(1) - front_threshold || 
             robot.adcRead(2) < robot.adcMD(2) - front_threshold) {
      robot.motor('A', -30); robot.motor('C', -30); delay(50);
      robot.motor('A', 0); robot.motor('C', 0); delay(100);
      set_f(2);
      break;
      }
    else {
      robot.motor('A', leftSpeed); robot.motor('C', rightSpeed);      
    }   
  }

  // เบรกนุ่มเพิ่มเติมเล็กน้อย (optional แต่แนะนำ)
  robot.motor('A', -3); robot.motor('C', -3);
  delay(10);
  robot.motor('A', 0); robot.motor('C', 0);

  // ============================= ส่วนหลังถึงระยะ: โหมด "line" =============================
  if (_line == "line") 
    {
      ch_line = true;
      Serial.println(">>> โหมด 'line' → เดินช้า PID ไจโร จนเจอเส้นหน้าแล้วหยุด");
      float slow_targetAngle = averageGyroZ(20);
      integral   = 0.0f;
      lastError  = 0.0f;
      lastTime   = millis();

      const int   slow_base_speed =10;       
      const float slow_kp         = 1.8f;    
      const int   min_speed       = 12;       
      const int   max_speed       =15;       
      const int   front_threshold = 60;      

      while (true) 
        {  
          delay(5);

          float currentAngle = gyro.gyro('z');
          float angleError = constrainAngle180(slow_targetAngle + currentAngle);  // แนะนำให้ใช้ - เหมือนด้านบน

          unsigned long now = millis();
          float dt = (now - lastTime) / 1000.0f;
          lastTime = now;
          dt = constrain(dt, 0.005f, 0.08f);

          integral += angleError * dt;
          integral = constrain(integral, -20, 20);  

          float derivative = (angleError - lastError) / dt;
          lastError = angleError;

          float correction = 1.3 * angleError + 0.001f * integral + 0.02f * derivative;

          // ใช้เครื่องหมายเดียวกับช่วงปกติ
          int leftSpeed  = slow_base_speed - (int)correction;
          int rightSpeed = slow_base_speed + (int)correction;

          leftSpeed  = constrain(leftSpeed,  min_speed, max_speed);
          rightSpeed = constrain(rightSpeed, min_speed, max_speed);

        // robot.motor('A', leftSpeed); robot.motor('C', rightSpeed);
          
          // หยุดเมื่อเจอเส้นหน้าดำ
          // ตรวจเส้นข้าง + เส้นหน้า
      if (robot.adcRead(0) < robot.adcMD(0) - side_threshold && 
          robot.adcRead(3) > robot.adcMD(3)) {
        robot.motor('A', leftSpeed*1.5); robot.motor('C', rightSpeed/3);
        gyro.resetAngles();
        Serial.println(">>> เจอเส้นข้างซ้าย → ปรับเลี้ยวขวา");
      } 
      else if (robot.adcRead(0) > robot.adcMD(0) && 
              robot.adcRead(3) < robot.adcMD(3) - side_threshold) {
        robot.motor('A', leftSpeed/3); robot.motor('C', rightSpeed*1.5);      
        gyro.resetAngles();
        Serial.println(">>> เจอเส้นข้างขวา → ปรับเลี้ยวซ้าย");
      } 
      
      else if (robot.adcRead(1) < robot.adcMD(1) - front_threshold || 
              robot.adcRead(2) < robot.adcMD(2) - front_threshold) {
                robot.motor('A', -30); robot.motor('C', -30); delay(50);
                robot.motor('A', 0); robot.motor('C', 0); delay(200);        
                break;                   
        
      }
      else {
        robot.motor('A', leftSpeed); robot.motor('C', rightSpeed);      
      } 

    }
  }
  else {
    Serial.printf(">>> โหมด '%s' → หยุดทันทีที่ระยะ %.1f cm\n", _line.c_str(), dist);
    robot.motor('A', -10); robot.motor('C', -10);
    delay(10);
    robot.motor('A', -1); robot.motor('C', -1);
    delay(30);
    ch_line = false;
  }

  _set_f = false;
  _set_b = false;

  float finalDist = abs(encoder1.Poss_R() - startPulsesL) / pulsesPerCm;
  //Serial.printf("=== move_fw END | ได้จริง: %.1f cm | เป้า: %.1f cm | Error: %.1f cm ===\n",finalDist, dist, finalDist - dist);
}


