// =============================================================================
// ฟังก์ชัน move_bw: ถอยหลังตามลู่ดำ 2 เส้นข้าง + ตรวจเส้นหลัง (adc1, adc2)
// PID เต็มรูปแบบทั้งช่วงปกติและช่วงช้า ๆ (โหมด "line")
// =============================================================================
void move_bw(int sl, float kp, float dist, String _line) {
  extern ArrayPico2 robot;
  extern EncoderLibrarys encoder;
  extern my_BMI160 gyro;

  char lr = ' ';  
  encoder.resetEncoders();
  _move_fw = false;  
  _move_bw = true;  

  // ปรับระยะถ้ามี flag จากการเดินหน้า
  if (_set_f == true) {
    dist += 5;
    _set_f = false;
  }

  long startPulsesL = encoder.Poss_L();

  gyro.resetAngles();
  delay(30);
  float initialAngle = averageGyroZ(20);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  Serial.printf("=== move_bw START | speed:%d | kp:%.2f | dist:%.1f cm | mode:%s ===\n", 
                sl, kp, dist, _line.c_str());

  const int side_threshold  = 30;
  const int front_threshold = 50;  // ใช้ตรวจ "เส้นหลัง" เมื่อถอย
  const int slow_base_speed = 6;
  const float slow_kp       = 1.0f;

  bool slowing = false;

  // ============================= ลูปถอยหลังหลักถึงระยะ =============================
  while (true) {
    delay(5);

    long currentPulses = encoder.Poss_L();
    long pulsesTraveled = abs(currentPulses - startPulsesL);
    float distanceTraveled = pulsesTraveled / pulsesPerCm;
    float remaining = dist - distanceTraveled;

    float slowZone = dist * 0.4f;
    if (remaining <= slowZone && !slowing) {
      slowing = true;
      //Serial.println(">>> เข้าโหมดช้าลงถอยหลัง");
    }
    int currentSpeed = slowing ? max((int)(sl * 0.6f), 15) : sl;

    if (remaining <= 1.5f) {
      robot.Motor(1, 1); delay(10);  // brake เล็กน้อย (ทิศตรงข้ามกับถอย)
      robot.Motor(0, 0);
      Serial.printf(">>> ถอยถึงระยะ %.1f cm แล้ว\n", dist);
      break;
    }

    // PID gyro (เหมือนเดิม)
    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(targetAngle - currentAngle);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    dt = constrain(dt, 0.005f, 0.08f);

    integral += angleError * dt;
    integral = constrain(integral, -60, 60);

    float derivative = (angleError - lastError) / dt;
    lastError = angleError;

    float correction = kp * angleError + 0.01f * integral + 0.2f * derivative;
    correction = constrain(correction, -30, 30);

    // ความเร็วถอยหลัง = ติดลบ
    int baseBackward = -currentSpeed;

    int leftSpeed  = baseBackward + (int)correction;   // สลับ correction เพราะถอยหลัง
    int rightSpeed = baseBackward - (int)correction;   // สลับซ้ายขวาให้ตรงกับการเลี้ยว

    leftSpeed  = constrain(leftSpeed, -100, -15);   // จำกัดไม่ให้เร็วเกินและไม่เป็นบวก
    rightSpeed = constrain(rightSpeed, -100, -15);

    // ตรวจเส้นข้าง (เมื่อถอยหลัง ซ้าย-ขวาจะสลับมุมมอง)
    if (robot.adcRead(4) < robot.adcMD(4) - side_threshold && 
        robot.adcRead(7) > robot.adcMD(7)) {
      // เจอเส้น "ซ้าย" (จากมุมถอย) → ปรับเลี้ยวขวา (ลดขวา)
      robot.Motor(leftSpeed, rightSpeed / 3);
      gyro.resetAngles();
      Serial.println(">>> เจอเส้นข้าง (ถอย) → ปรับ");
    } 
    else if (robot.adcRead(4) > robot.adcMD(4) && 
             robot.adcRead(7) < robot.adcMD(7) - side_threshold) {
      robot.Motor(leftSpeed / 2, rightSpeed);
      gyro.resetAngles();
      Serial.println(">>> เจอเส้นข้างขวา (ถอย) → ปรับ");
    } 
    else if (robot.adcRead(5) < robot.adcMD(5) - front_threshold || 
             robot.adcRead(6) < robot.adcMD(6) - front_threshold) {
      Serial.println(">>> เจอเส้นหลังระหว่างถอย → จัดการ");
      // Logic จัดการเส้นหลัง (คล้ายเดิม แต่ถอยต่อหรือหยุดตามต้องการ)
      robot.Motor(30, 30); delay(30);  // brake ตรงข้าม
      robot.Motor(1, 1); delay(10);
      robot.Motor(0, 0); delay(50);
      break;
    }
    else {
      robot.Motor(leftSpeed, rightSpeed);
    }
  }

  // ============================= ส่วนหลังถึงระยะ: โหมด "line" ถอยช้า + PID ไจโร + หยุดเมื่อเจอเส้นหลัง =============================
  if (_line == "line") {
    Serial.println(">>> โหมด 'line' (ถอย) → ถอยช้า PID ไจโร จนเจอเส้นหลังแล้วหยุด");

    gyro.resetAngles();
    delay(30);
    float slow_targetAngle = averageGyroZ(20);
    integral   = 0.0f;
    lastError  = 0.0f;
    lastTime   = millis();

    const int   slow_base_speed = 6;
    const float slow_kp         = 1.0f;
    const int   min_speed       = 3;
    const int   max_speed       = 9;
    const int   front_threshold = 60;

    while (true) {
      delay(5);

      float currentAngle = gyro.gyro('z');
      float angleError = constrainAngle180(slow_targetAngle - currentAngle);

      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0f;
      lastTime = now;
      dt = constrain(dt, 0.005f, 0.08f);

      integral += angleError * dt;
      integral = constrain(integral, -20, 20);

      float derivative = (angleError - lastError) / dt;
      lastError = angleError;

      float correction = slow_kp * angleError + 0.0001f * integral + 0.002f * derivative;

      int baseBackward = -slow_base_speed;
      int leftSpeed  = baseBackward - (int)correction;
      int rightSpeed = baseBackward + (int)correction;

      leftSpeed  = constrain(leftSpeed,  -max_speed, -min_speed);
      rightSpeed = constrain(rightSpeed, -max_speed, -min_speed);

      robot.Motor(leftSpeed, rightSpeed);

      // หยุดเมื่อเจอเส้นหลัง (adc1, adc2)
      if (robot.adcRead(5) < robot.adcMD(5) - front_threshold ||
          robot.adcRead(6) < robot.adcMD(6) - front_threshold) {
        robot.Motor(30, 30); delay(20);
        robot.Motor(10, 10); delay(200);   
        while(1)
              {
                robot.Motor(15, 15);
                if(robot.adcRead(5) > robot.adcMD(5) && robot.adcRead(6) > robot.adcMD(6))
                  {
                    break;
                  }
              }
            while(1)
              {
                robot.Motor(15, 15);
                if(robot.adcRead(7) > robot.adcMD(7) || robot.adcRead(4) > robot.adcMD(4))
                  {
                    robot.Motor(-5, -5); delay(10);
                    break;
                  }
              }
        set_b(2);
        ch_line = true;
        delay(100);
        break;
      }
    }
  }
  else {
    Serial.printf(">>> โหมด '%s' → หยุดทันทีหลังถอย %.1f cm\n", _line.c_str(), dist);
    robot.Motor(25, 25); delay(30);
    robot.Motor(1, 1); delay(30);
    robot.Motor(0, 0);
    ch_line = false;
  }

  _set_f = false;
  _set_b = false;

  float finalDist = abs(encoder.Poss_L() - startPulsesL) / pulsesPerCm;
  Serial.printf("=== move_bw END | ได้จริง: %.1f cm | เป้า: %.1f cm | Error: %.1f cm ===\n", 
                finalDist, dist, finalDist - dist);
}