void move_bw(int sl, float kp, float dist, String _line) {
  
  extern PiperPico2 robot;
  extern my_BMI160 gyro;
  extern EncoderLibrarys encoder1;
  extern EncoderLibraryss encoder2;

  encoder1.resetEncoders();
  _move_fw = false;  
  _move_bw = true;  

  if (_set_b == true) {
    dist += 5;           // ถ้ามี offset เดิม ก็ยังใช้เหมือนกัน
    _set_b = false;
  }

  float targetPulses = dist * pulsesPerCm;
  long startPulsesL = encoder1.Poss_R();   // ใช้ encoder เดียวกัน แต่ค่าจะติดลบตอนถอย

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

    long currentPulses = encoder1.Poss_R();
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
      robot.motor('A', 2); robot.motor('C', 2);   // เบรกสวนทาง (forward เล็กน้อย)
      delay(10);
      robot.motor('A', 0); robot.motor('C', 0);
      
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

    // ==============================================
    // ตรวจเซ็นเซอร์ - ปรับ logic ให้เหมาะกับการถอยหลัง
    // ==============================================
    // หมายเหตุ: เส้นข้างอาจสลับซ้าย-ขวา หรือต้องทดสอบจริงว่าตรงกับการถอยหรือไม่
    if (robot.adcRead(4) < robot.adcMD(4) - side_threshold && 
        robot.adcRead(7) > robot.adcMD(7)) {
      // เส้นข้างซ้าย (ปรับการเลี้ยวให้เหมาะกับถอย)
      robot.motor('A', leftSpeed*0.7);   // ช้าลงฝั่งนี้
      robot.motor('C', rightSpeed*1.4);  // เร่งอีกฝั่ง
      gyro.resetAngles();
      Serial.println(">>> (ถอย) เจอเส้นข้างซ้าย → ปรับเลี้ยว");
    } 
    else if (robot.adcRead(4) > robot.adcMD(4) && 
             robot.adcRead(7) < robot.adcMD(7) - side_threshold) {
      robot.motor('A', leftSpeed*1.4);
      robot.motor('C', rightSpeed*0.7);
      gyro.resetAngles();
      Serial.println(">>> (ถอย) เจอเส้นข้างขวา → ปรับเลี้ยว");
    } 
    // สำหรับเส้นหน้าตอนถอย → จริง ๆ คือ "ด้านหลัง" ของหุ่น
    else if (robot.adcRead(5) < robot.adcMD(5) - front_threshold || 
             robot.adcRead(6) < robot.adcMD(6) - front_threshold) {
      Serial.println(">>> (ถอย) เจอสิ่งกีดขวางด้านหลัง → หยุด/จัดการ");
      robot.motor('A', 40); robot.motor('C', 40); delay(40);   // ไปข้างหน้าเล็กน้อยเพื่อหนี
      robot.motor('A', 0); robot.motor('C', 0);
      break;  // หรือจะใส่ logic หมุน/ถอยต่อก็ได้ ขึ้นกับความต้องการ
    }
    else {
      robot.motor('A', leftSpeed);
      robot.motor('C', rightSpeed);      
    }   
  }

  // เบรกนุ่มทิ้งท้าย
  robot.motor('A', 3); robot.motor('C', 3);   // สวนทางเล็กน้อย
  delay(30);
  robot.motor('A', 0); robot.motor('C', 0);

  // ============================= ส่วนหลังถึงระยะ: โหมด "line" =============================
  if (_line == "line") {
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

    while (true) {  
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
    if (robot.adcRead(4) < robot.adcMD(4) - side_threshold && 
        robot.adcRead(7) > robot.adcMD(7)) {
      robot.motor('A', -leftSpeed); robot.motor('C', -(rightSpeed/3));
      gyro.resetAngles();
      Serial.println(">>> เจอเส้นข้างซ้าย → ปรับเลี้ยวขวา");
    } 
    else if (robot.adcRead(4) > robot.adcMD(4) && 
             robot.adcRead(7) < robot.adcMD(7) - side_threshold) {
      robot.motor('A', -(leftSpeed/3)); robot.motor('C', -rightSpeed);      
      gyro.resetAngles();
      Serial.println(">>> เจอเส้นข้างขวา → ปรับเลี้ยวซ้าย");
    } 
    else if (robot.adcRead(5) < robot.adcMD(5) - front_threshold || 
             robot.adcRead(6) < robot.adcMD(6) - front_threshold) {
      Serial.println(">>> เจอตะกียบระหว่างทาง → จัดการถอย/หมุน");
      robot.motor('A', 30); robot.motor('C', 30); delay(30);
      robot.motor('A', 1); robot.motor('C', 1); delay(10);
      robot.motor('A', 0); robot.motor('C', 0); delay(200);
      set_b(2);
            ch_line = true;
            break;
    }
    else {
      robot.motor('A', -leftSpeed); robot.motor('C', -rightSpeed);      
    } 

    }
  }
  else {
    Serial.printf(">>> โหมด '%s' → หยุดทันทีที่ระยะ %.1f cm\n", _line.c_str(), dist);
    robot.motor('A', 20); robot.motor('C', 20);
    delay(40);
    robot.motor('A', 1); robot.motor('C', 1);
    delay(30);
    ch_line = false;
  }

  _set_f = false;
  _set_b = false;

  float finalDist = abs(encoder1.Poss_R() - startPulsesL) / pulsesPerCm;
  Serial.printf("=== move_fw END | ได้จริง: %.1f cm | เป้า: %.1f cm | Error: %.1f cm ===\n", 
                finalDist, dist, finalDist - dist);
}
