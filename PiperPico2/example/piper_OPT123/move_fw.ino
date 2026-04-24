void move_fw(int sl, float kp, float dist, String _line) {
  
  extern PiperPico2 robot;
  extern my_BMI160 gyro;
  extern EncoderLibrarys encoder1;
  extern EncoderLibraryss encoder2;

  char lr = ' ';  // Safety init เพื่อป้องกัน undefined behavior
  encoder1.resetEncoders();
  _move_fw = true;  
  _move_bw = false;  

  // ปรับระยะถ้ามี flag จากการถอยก่อนหน้า
  if (_set_b == true) {
    dist += 5;
    _set_b = false;
  }


  float targetPulses = dist * pulsesPerCm;
  long startPulsesL = encoder1.Poss_L();

  gyro.resetAngles();
  delay(30);
  float initialAngle = averageGyroZ(10);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  Serial.printf("=== move_fw | speed: %d | kp: %.2f | dist: %.1f cm ===\n", sl, kp, dist);

   // กำหนดค่าคงที่ทั้งหมดที่นี่ (แก้ error 'not declared in this scope')
  // ==============================================
  const int side_threshold  = 30;   // threshold สำหรับเส้นข้าง (adc0, adc3)
  const int front_threshold = 50;   // threshold สำหรับเส้นหน้าตะกียบ (adc1, adc2)
  const int slow_base_speed = 18;   // ความเร็วพื้นฐานในโหมดช้า ๆ ("line")
  const float slow_kp       = 1.5f; // PID gain สำหรับช่วงช้า (sensitive กว่า)

  bool slowing = false;

  while (true) {
    delay(5);  // loop ถี่ขึ้นมาก
    long currentPulses = encoder1.Poss_L();
    long pulsesTraveled = abs(currentPulses - startPulsesL);
    float distanceTraveled = pulsesTraveled / pulsesPerCm;

    float remaining = dist - distanceTraveled;

    // เริ่มช้าลงเมื่อเหลือ 20% ของระยะทั้งหมด
    float slowZone = dist * 0.3f;
    if (remaining <= slowZone && !slowing) {
      slowing = true;
      Serial.println("เข้าโหมดช้าลง 30% (เหลือ 20% ของระยะ)");
    }

    int currentSpeed = sl;
    if (slowing) {
      currentSpeed = sl * 0.6f;  // ลดลง 30%
      currentSpeed = max(currentSpeed, 20);  // ความเร็วขั้นต่ำ
    }

    if (remaining <= 1.5f) {
      robot.motor('A', 0); robot.motor('C', 0);
      Serial.println("ถึงเป้าแล้ว (หยุดเรียบร้อย)");
      break;
    }

    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(targetAngle + currentAngle);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt <= 0.001f) dt = 0.02f;
    if (dt > 0.08f) dt = 0.02f;

    integral += angleError * dt;
    integral = constrain(integral, -60, 60);

    float derivative = (angleError - lastError) / dt;
    lastError = angleError;

    float correction = kp * angleError + 0.01f * integral + 0.2f * derivative;
    correction = constrain(correction, -30, 30);

    int leftSpeed  = currentSpeed + correction;
    int rightSpeed = currentSpeed - correction;

    leftSpeed  = constrain(leftSpeed, -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);
    
    // ตรวจเส้นข้าง + เส้นหน้า (ใช้ const ที่ประกาศด้านบน)
    if (robot.adcRead(0) < robot.adcMD(0) - side_threshold && 
        robot.adcRead(3) > robot.adcMD(3)) {
      robot.motor('A', leftSpeed); robot.motor('C', rightSpeed/3);
      gyro.resetAngles();
      Serial.println(">>> เจอเส้นข้างซ้าย → ปรับเลี้ยวขวา");
    } 
    else if (robot.adcRead(0) > robot.adcMD(0) && 
             robot.adcRead(3) < robot.adcMD(3) - side_threshold) {
      robot.motor('A', leftSpeed/3); robot.motor('C', rightSpeed);      
      gyro.resetAngles();
      Serial.println(">>> เจอเส้นข้างขวา → ปรับเลี้ยวซ้าย");
    } 
    else if (robot.adcRead(1) < robot.adcMD(1) - front_threshold || 
             robot.adcRead(2) < robot.adcMD(2) - front_threshold) {
      Serial.println(">>> เจอตะกียบระหว่างทาง → จัดการถอย/หมุน");
      // Logic เดิมของคุณสำหรับจัดการตะกียบ (ถอย fw_to_rotate)
      robot.motor('A', -30); robot.motor('C', -30);delay(30);
      robot.motor('A', -1); robot.motor('C', -1); delay(10);

      // ... (ใส่ logic while(1) สำหรับปรับทิศทางตาม lr เดิมของคุณที่นี่)
      // ตัวอย่างย่อ:
      /**/
      encoder1.resetEncoders();
      do {
        robot.motor('A', -20); robot.motor('C', -20);
      } while (encoder1.Poss_L() > -fw_to_rotate);
      robot.motor('A', 0); robot.motor('C', 0); delay(50);
      break;  // ออกจากลูปหลัก
    }
    else {
      robot.motor('A', leftSpeed); robot.motor('C', rightSpeed);      
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
    const int   slow_base_speed = 20;       // เริ่มจาก 6 ถ้ายังเร็วให้ลดเหลือ 5 หรือ 4
    const float slow_kp         = 1.0f;    // PID ไม่แรงมาก เพราะช้าและระยะใกล้
    const int   min_speed       = 20;       // ต่ำสุดให้ยังขยับได้นิดหน่อย
    const int   max_speed       = 50;       // จำกัดสูงสุดไม่ให้เกิน 9 (ช้าจริง)
    const int   front_threshold = 60;      // เพิ่มนิดเพื่อให้ตรวจจับเส้นได้เร็วขึ้น

    while (true) {  // วนจนกว่าจะเจอเส้น ไม่ต้อง timeout เพราะระยะใกล้อยู่แล้ว
        delay(5);

        // PID จากไจโรเท่านั้น
        float currentAngle = gyro.gyro('z');
        float angleError = constrainAngle180(slow_targetAngle + currentAngle);

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
        robot.motor('A', leftSpeed); robot.motor('C', rightSpeed);
        

        // หยุดเมื่อเจอเส้นหน้าดำ (อย่างน้อยหนึ่งข้าง)
        if (robot.adcRead(1) < robot.adcMD(1) - front_threshold ||
            robot.adcRead(2) < robot.adcMD(2) - front_threshold) {
            robot.motor('A', -10); robot.motor('C', -10);
            delay(30);
            robot.motor('A', -1); robot.motor('C', -1); delay(30);
            ch_line = true;
            //delay(100);  // รอให้มั่นใจว่าไม่ไหลต่อ (ปรับได้ตาม inertia ของหุ่น)
            break;
        }
    }
}
else {
    // โหมดอื่น (เช่น "none_line") → หยุดทันทีเหมือนเดิม
    Serial.printf(">>> โหมด '%s' → หยุดทันทีที่ระยะ %.1f cm\n", _line.c_str(), dist);
    robot.motor('A', -25); robot.motor('C', -25);
     delay(30);
     robot.motor('A', -1); robot.motor('C', -1);
     delay(30);
    ch_line = false;
}

  _set_f = false;
  _set_b = false;

  float finalDist = abs(encoder1.Poss_L() - startPulsesL) / pulsesPerCm;
  Serial.printf("=== move_fw END | ได้จริง: %.1f cm | เป้า: %.1f cm | Error: %.1f cm ===\n", 
                finalDist, dist, finalDist - dist);
}