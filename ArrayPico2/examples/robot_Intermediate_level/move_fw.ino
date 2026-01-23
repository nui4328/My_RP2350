// =============================================================================
// ฟังก์ชัน move_fw: เดินตรงตามลู่ดำ 2 เส้นข้าง + ตรวจตะกียบหน้า
// PID เต็มรูปแบบทั้งช่วงปกติและช่วงช้า ๆ (โหมด "line")
// ผู้พัฒนา: Grok ปรับให้ครูชัยวัฒน์ (แก้ error 'not declared in this scope')
// =============================================================================
void move_fw(int sl, float kp, float dist, String _line) {
  extern ArrayPico2 robot;
  extern EncoderLibrarys encoder;
  extern my_BMI160 gyro;

  char lr = ' ';  // Safety init เพื่อป้องกัน undefined behavior
  encoder.resetEncoders();
  _move_fw = true;  
  _move_bw = false;  

  // ปรับระยะถ้ามี flag จากการถอยก่อนหน้า
  if (_set_b == true) {
    dist += 5;
    _set_b = false;
  }

  long startPulsesL = encoder.Poss_L();

  gyro.resetAngles();
  delay(30);
  float initialAngle = averageGyroZ(20);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  Serial.printf("=== move_fw START | speed:%d | kp:%.2f | dist:%.1f cm | mode:%s ===\n", 
                sl, kp, dist, _line.c_str());

  // ==============================================
  // กำหนดค่าคงที่ทั้งหมดที่นี่ (แก้ error 'not declared in this scope')
  // ==============================================
  const int side_threshold  = 30;   // threshold สำหรับเส้นข้าง (adc0, adc3)
  const int front_threshold = 50;   // threshold สำหรับเส้นหน้าตะกียบ (adc1, adc2)
  const int slow_base_speed = 18;   // ความเร็วพื้นฐานในโหมดช้า ๆ ("line")
  const float slow_kp       = 1.5f; // PID gain สำหรับช่วงช้า (sensitive กว่า)

  bool slowing = false;

  // ============================= ลูปเดินหลักถึงระยะที่ตั้ง (dist) =============================
  while (true) {
    delay(5);  // ~200Hz loop

    long currentPulses = encoder.Poss_L();
    long pulsesTraveled = abs(currentPulses - startPulsesL);
    float distanceTraveled = pulsesTraveled / pulsesPerCm;
    float remaining = dist - distanceTraveled;

    // เข้าโหมดช้าลงเมื่อเหลือ 30% ของระยะ
    float slowZone = dist * 0.4f;
    if (remaining <= slowZone && !slowing) {
      slowing = true;
      //Serial.println(">>> เข้าโหมดช้าลง 40% (เหลือ ~30% ของระยะ)");
    }
    int currentSpeed = slowing ? max((int)(sl * 0.6f), 15) : sl;

    if (remaining <= 1.5f) {
        robot.Motor(-1, -1);
        delay(10);
        Serial.printf(">>> ถึงระยะ %.1f cm แล้ว (หยุดเรียบร้อย)\n", dist);
        break;
    }

    // PID จาก gyro
    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(targetAngle - currentAngle);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    dt = constrain(dt, 0.005f, 0.08f);  // ป้องกัน dt ผิดปกติ

    integral += angleError * dt;
    integral = constrain(integral, -60, 60);

    float derivative = (angleError - lastError) / dt;
    lastError = angleError;

    float correction = kp * angleError + 0.01f * integral + 0.2f * derivative;
    correction = constrain(correction, -30, 30);

    int leftSpeed  = currentSpeed + (int)correction;
    int rightSpeed = currentSpeed - (int)correction;

    leftSpeed  = constrain(leftSpeed, -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);

    // ตรวจเส้นข้าง + เส้นหน้า (ใช้ const ที่ประกาศด้านบน)
    if (robot.adcRead(0) < robot.adcMD(0) - side_threshold && 
        robot.adcRead(3) > robot.adcMD(3)) {
      robot.Motor(leftSpeed, rightSpeed / 3);
      gyro.resetAngles();
      Serial.println(">>> เจอเส้นข้างซ้าย → ปรับเลี้ยวขวา");
    } 
    else if (robot.adcRead(0) > robot.adcMD(0) && 
             robot.adcRead(3) < robot.adcMD(3) - side_threshold) {
      robot.Motor(leftSpeed / 3, rightSpeed);
      gyro.resetAngles();
      Serial.println(">>> เจอเส้นข้างขวา → ปรับเลี้ยวซ้าย");
    } 
    else if (robot.adcRead(1) < robot.adcMD(1) - front_threshold || 
             robot.adcRead(2) < robot.adcMD(2) - front_threshold) {
      Serial.println(">>> เจอตะกียบระหว่างทาง → จัดการถอย/หมุน");
      // Logic เดิมของคุณสำหรับจัดการตะกียบ (ถอย fw_to_rotate)
      robot.Motor(-30, -30); delay(30);
      robot.Motor(-1, -1); delay(10);

      // ... (ใส่ logic while(1) สำหรับปรับทิศทางตาม lr เดิมของคุณที่นี่)
      // ตัวอย่างย่อ:
      /**/
      encoder.resetEncoders();
      do {
        robot.Motor(-20, -20);
      } while (encoder.Poss_L() > -fw_to_rotate);
      robot.Motor(0, 0); delay(50);
      break;  // ออกจากลูปหลัก
    }
    else {
      robot.Motor(leftSpeed, rightSpeed);
    }
  }

 // ============================= ส่วนหลังถึงระยะ: โหมด "line" เดินช้า + PID ไจโร + หยุดเมื่อเจอเส้นหน้า =============================
if (_line == "line") {
    Serial.println(">>> โหมด 'line' → เดินช้า PID ไจโร จนเจอเส้นหน้าแล้วหยุด");
    float slow_targetAngle = averageGyroZ(20);
    integral   = 0.0f;
    lastError  = 0.0f;
    lastTime   = millis();

    // ค่าคงที่สำหรับช่วงช้า ๆ (ปรับตามทดสอบจริง)
    const int   slow_base_speed = 6;       // เริ่มจาก 6 ถ้ายังเร็วให้ลดเหลือ 5 หรือ 4
    const float slow_kp         = 1.0f;    // PID ไม่แรงมาก เพราะช้าและระยะใกล้
    const int   min_speed       = 3;       // ต่ำสุดให้ยังขยับได้นิดหน่อย
    const int   max_speed       = 9;       // จำกัดสูงสุดไม่ให้เกิน 9 (ช้าจริง)
    const int   front_threshold = 60;      // เพิ่มนิดเพื่อให้ตรวจจับเส้นได้เร็วขึ้น

    while (true) {  // วนจนกว่าจะเจอเส้น ไม่ต้อง timeout เพราะระยะใกล้อยู่แล้ว
        delay(5);

        // PID จากไจโรเท่านั้น
        float currentAngle = gyro.gyro('z');
        float angleError = constrainAngle180(slow_targetAngle - currentAngle);

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        dt = constrain(dt, 0.005f, 0.08f);

        integral += angleError * dt;
        integral = constrain(integral, -20, 20);  // windup น้อยลงเพราะช้า

        float derivative = (angleError - lastError) / dt;
        lastError = angleError;

        float correction = 1.0 * angleError + 0.0001f * integral + 0.002f * derivative;

        int leftSpeed  = slow_base_speed + (int)correction;
        int rightSpeed = slow_base_speed - (int)correction;

        leftSpeed  = constrain(leftSpeed,  min_speed, max_speed);
        rightSpeed = constrain(rightSpeed, min_speed, max_speed);

        // ส่งความเร็ว
        robot.Motor(leftSpeed, rightSpeed);

        // หยุดเมื่อเจอเส้นหน้าดำ (อย่างน้อยหนึ่งข้าง)
        if (robot.adcRead(1) < robot.adcMD(1) - front_threshold ||
            robot.adcRead(2) < robot.adcMD(2) - front_threshold) {
            
            robot.Motor(-10, -10);delay(30);
            robot.Motor(-1, -1); delay(30);
            ch_line = true;
            //delay(100);  // รอให้มั่นใจว่าไม่ไหลต่อ (ปรับได้ตาม inertia ของหุ่น)
            break;
        }
    }
}
else {
    // โหมดอื่น (เช่น "none_line") → หยุดทันทีเหมือนเดิม
    Serial.printf(">>> โหมด '%s' → หยุดทันทีที่ระยะ %.1f cm\n", _line.c_str(), dist);
    robot.Motor(-25, -25); delay(30);
    robot.Motor(-1, -1); delay(30);
    ch_line = false;
}

  _set_f = false;
  _set_b = false;

  float finalDist = abs(encoder.Poss_L() - startPulsesL) / pulsesPerCm;
  Serial.printf("=== move_fw END | ได้จริง: %.1f cm | เป้า: %.1f cm | Error: %.1f cm ===\n", 
                finalDist, dist, finalDist - dist);
}

// =============================================================================
// ฟังก์ชัน move_fw: เดินตรงตามลู่ดำ 2 เส้นข้าง + ตรวจตะกียบหน้า
// PID เต็มรูปแบบทั้งช่วงปกติและช่วงช้า ๆ (โหมด "line")
// ผู้พัฒนา: Grok ปรับให้ครูชัยวัฒน์ (แก้ error 'not declared in this scope')
// =============================================================================
void chopsticks_fw(int sl, float kp, float dist, String _line) {
  extern ArrayPico2 robot;
  extern EncoderLibrarys encoder;
  extern my_BMI160 gyro;

  char lr = ' ';  // Safety init เพื่อป้องกัน undefined behavior
  encoder.resetEncoders();
  _move_fw = true;  
  _move_bw = false;  

  // ปรับระยะถ้ามี flag จากการถอยก่อนหน้า
  if (_set_b == true) {
    dist += 5;
    _set_b = false;
  }

  long startPulsesL = encoder.Poss_L();

  gyro.resetAngles();
  delay(30);
  float initialAngle = averageGyroZ(20);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  Serial.printf("=== move_fw START | speed:%d | kp:%.2f | dist:%.1f cm | mode:%s ===\n", 
                sl, kp, dist, _line.c_str());

  // ==============================================
  // กำหนดค่าคงที่ทั้งหมดที่นี่ (แก้ error 'not declared in this scope')
  // ==============================================
  const int side_threshold  = 30;   // threshold สำหรับเส้นข้าง (adc0, adc3)
  const int front_threshold = 50;   // threshold สำหรับเส้นหน้าตะกียบ (adc1, adc2)
  const int slow_base_speed = 18;   // ความเร็วพื้นฐานในโหมดช้า ๆ ("line")
  const float slow_kp       = 1.5f; // PID gain สำหรับช่วงช้า (sensitive กว่า)

  bool slowing = false;

  // ============================= ลูปเดินหลักถึงระยะที่ตั้ง (dist) =============================
  while (true) {
    delay(5);  // ~200Hz loop

    long currentPulses = encoder.Poss_L();
    long pulsesTraveled = abs(currentPulses - startPulsesL);
    float distanceTraveled = pulsesTraveled / pulsesPerCm;
    float remaining = dist - distanceTraveled;

    // เข้าโหมดช้าลงเมื่อเหลือ 30% ของระยะ
    float slowZone = dist * 0.4f;
    if (remaining <= slowZone && !slowing) {
      slowing = true;
      //Serial.println(">>> เข้าโหมดช้าลง 40% (เหลือ ~30% ของระยะ)");
    }
    int currentSpeed = slowing ? max((int)(sl * 0.6f), 15) : sl;

    if (remaining <= 1.5f) {
        robot.Motor(-1, -1);
        delay(10);
        Serial.printf(">>> ถึงระยะ %.1f cm แล้ว (หยุดเรียบร้อย)\n", dist);
        break;
    }

    // PID จาก gyro
    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(targetAngle - currentAngle);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    dt = constrain(dt, 0.005f, 0.08f);  // ป้องกัน dt ผิดปกติ

    integral += angleError * dt;
    integral = constrain(integral, -60, 60);

    float derivative = (angleError - lastError) / dt;
    lastError = angleError;

    float correction = kp * angleError + 0.01f * integral + 0.2f * derivative;
    correction = constrain(correction, -30, 30);

    int leftSpeed  = currentSpeed + (int)correction;
    int rightSpeed = currentSpeed - (int)correction;

    leftSpeed  = constrain(leftSpeed, -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);
    robot.Motor(leftSpeed, rightSpeed);
    
  }

 // ============================= ส่วนหลังถึงระยะ: โหมด "line" เดินช้า + PID ไจโร + หยุดเมื่อเจอเส้นหน้า =============================
if (_line == "line") {
    Serial.println(">>> โหมด 'line' → เดินช้า PID ไจโร จนเจอเส้นหน้าแล้วหยุด");
    float slow_targetAngle = averageGyroZ(20);
    integral   = 0.0f;
    lastError  = 0.0f;
    lastTime   = millis();

    // ค่าคงที่สำหรับช่วงช้า ๆ (ปรับตามทดสอบจริง)
    const int   slow_base_speed = 6;       // เริ่มจาก 6 ถ้ายังเร็วให้ลดเหลือ 5 หรือ 4
    const float slow_kp         = 1.0f;    // PID ไม่แรงมาก เพราะช้าและระยะใกล้
    const int   min_speed       = 3;       // ต่ำสุดให้ยังขยับได้นิดหน่อย
    const int   max_speed       = 9;       // จำกัดสูงสุดไม่ให้เกิน 9 (ช้าจริง)
    const int   front_threshold = 60;      // เพิ่มนิดเพื่อให้ตรวจจับเส้นได้เร็วขึ้น

    while (true) {  // วนจนกว่าจะเจอเส้น ไม่ต้อง timeout เพราะระยะใกล้อยู่แล้ว
        delay(5);

        // PID จากไจโรเท่านั้น
        float currentAngle = gyro.gyro('z');
        float angleError = constrainAngle180(slow_targetAngle - currentAngle);

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        dt = constrain(dt, 0.005f, 0.08f);

        integral += angleError * dt;
        integral = constrain(integral, -20, 20);  // windup น้อยลงเพราะช้า

        float derivative = (angleError - lastError) / dt;
        lastError = angleError;

        float correction = 1.0 * angleError + 0.0001f * integral + 0.002f * derivative;

        int leftSpeed  = slow_base_speed + (int)correction;
        int rightSpeed = slow_base_speed - (int)correction;

        leftSpeed  = constrain(leftSpeed,  min_speed, max_speed);
        rightSpeed = constrain(rightSpeed, min_speed, max_speed);

        // ส่งความเร็ว
        robot.Motor(leftSpeed, rightSpeed);

        // หยุดเมื่อเจอเส้นหน้าดำ (อย่างน้อยหนึ่งข้าง)
        if (robot.adcRead(1) < robot.adcMD(1) - front_threshold ||
            robot.adcRead(2) < robot.adcMD(2) - front_threshold) {
            
            robot.Motor(-10, -10);delay(30);
            robot.Motor(-1, -1); delay(30);
            ch_line = true;
            //delay(100);  // รอให้มั่นใจว่าไม่ไหลต่อ (ปรับได้ตาม inertia ของหุ่น)
            break;
        }
    }
}
else {
    // โหมดอื่น (เช่น "none_line") → หยุดทันทีเหมือนเดิม
    Serial.printf(">>> โหมด '%s' → หยุดทันทีที่ระยะ %.1f cm\n", _line.c_str(), dist);
    robot.Motor(-25, -25); delay(30);
    robot.Motor(-1, -1); delay(30);
    ch_line = false;
}

  _set_f = false;
  _set_b = false;

  float finalDist = abs(encoder.Poss_L() - startPulsesL) / pulsesPerCm;
  Serial.printf("=== move_fw END | ได้จริง: %.1f cm | เป้า: %.1f cm | Error: %.1f cm ===\n", 
                finalDist, dist, finalDist - dist);
}