void move_limit_sw(int sl, float kp, int targetADC) {
  extern PiperPico2 robot;
  extern EncoderLibrarys encoder;  // ยังคงไว้เผื่อใช้ในอนาคต

  // ===== เริ่มต้น =====
  gyro.resetAngles();
  delay(30);
  float initialAngle = averageGyroZ(10);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  Serial.printf("=== move_adc | speed: %d | kp: %.2f | targetADC: %d ===\n", 
                sl, kp, targetADC);

  while (true) {
    delay(5);

    int currentADC = robot.adcRead(5);

    // หยุดเมื่อถึงเป้าหมาย (มี deadband ป้องกันการสั่น)
    if (abs(currentADC) >= targetADC) {     // ปรับ 8 ได้ตามความเสถียรของเซ็นเซอร์
      break;
    }

    // === Gyro PID สำหรับรักษาทิศทาง ===
    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(targetAngle + currentAngle);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt <= 0.001f) dt = 0.02f;
    if (dt > 0.08f) dt = 0.02f;

    integral += angleError * dt;
    integral = constrain(integral, -60.0f, 60.0f);

    float derivative = (angleError - lastError) / dt;
    lastError = angleError;

    float correction = kp * angleError + 0.01f * integral + 0.2f * derivative;
    correction = constrain(correction, -30, 30);

    int leftSpeed  = sl - correction;
    int rightSpeed = sl + correction;

    leftSpeed  = constrain(leftSpeed,  -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);

    robot.motor('A', leftSpeed);
    robot.motor('C', rightSpeed);
  }

  // ===== หยุดด้วย brake เบา ๆ เพื่อลด inertia =====
  robot.motor('A', -20); 
  robot.motor('C', -20);
  delay(15);
  robot.motor('A', -1); 
  robot.motor('C', -1);
  delay(30);
  robot.motor('A', -20); 
  robot.motor('C', -20);
  delay(65);
  robot.motor('A', -1); 
  robot.motor('C', -1);
  delay(60);

  int finalADC = robot.adcRead(3);
  Serial.printf("สิ้นสุด move_adc → ADC สุดท้าย: %d (เป้า: %d) Error: %d\n", 
                finalADC, targetADC, finalADC - targetADC);
}

void move_adc(int sl, float kp, int targetADC) {
  extern PiperPico2 robot;
  extern EncoderLibrarys encoder;  // ยังคงไว้เผื่อใช้ในอนาคต

  // ===== เริ่มต้น =====
  gyro.resetAngles();
  delay(30);
  float initialAngle = averageGyroZ(10);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  Serial.printf("=== move_adc | speed: %d | kp: %.2f | targetADC: %d ===\n", 
                sl, kp, targetADC);

  while (true) {
    delay(5);

    int currentADC = robot.adcRead(3);

    // หยุดเมื่อถึงเป้าหมาย (มี deadband ป้องกันการสั่น)
    if (abs(currentADC) >= targetADC) {     // ปรับ 8 ได้ตามความเสถียรของเซ็นเซอร์
      break;
    }

    // === Gyro PID สำหรับรักษาทิศทาง ===
    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(targetAngle + currentAngle);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt <= 0.001f) dt = 0.02f;
    if (dt > 0.08f) dt = 0.02f;

    integral += angleError * dt;
    integral = constrain(integral, -60.0f, 60.0f);

    float derivative = (angleError - lastError) / dt;
    lastError = angleError;

    float correction = kp * angleError + 0.01f * integral + 0.2f * derivative;
    correction = constrain(correction, -30, 30);

    int leftSpeed  = sl - correction;
    int rightSpeed = sl + correction;

    leftSpeed  = constrain(leftSpeed,  -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);

    robot.motor('A', leftSpeed);
    robot.motor('C', rightSpeed);
    if (robot.adcRead(5) > 1500)
      {
        break;
      }
  }

  // ===== หยุดด้วย brake เบา ๆ เพื่อลด inertia =====
  robot.motor('A', -10); 
  robot.motor('C', -10);
  delay(25);
  robot.motor('A', -1); 
  robot.motor('C', -1);
  delay(60);

  int finalADC = robot.adcRead(3);
  Serial.printf("สิ้นสุด move_adc → ADC สุดท้าย: %d (เป้า: %d) Error: %d\n", 
                finalADC, targetADC, finalADC - targetADC);
}

void move_stuck(int sl, float kp, float dist) {
  extern PiperPico2 robot;

  if (dist <= 0.0f) return;
  if (sl <= 0) sl = 40;

  float targetPulses = dist * pulsesPerCm;
  long startPulsesL = encoder1.Poss_L();

  gyro.resetAngles();
  delay(50);

  float initialAngle = averageGyroZ(20);
  float targetAngle = initialAngle;

  // PID สำหรับควบคุมทิศทาง
  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  float Kp = kp;
  float Ki = 0.09f;     // ปรับได้ตามการทดสอบ
  float Kd = 0.13f;

  Serial.printf("=== move_fw | speed: %d | Kp: %.2f | dist: %.1f cm | (มีระบบตรวจชน) ===\n", 
                sl, Kp, dist);

  // === ตัวแปรสำหรับตรวจจับการชนด้วย encoder ===
  long lastPulsesL = startPulsesL;
  unsigned long lastMoveTime = millis();
  const unsigned long STUCK_TIME_MS = 140;     // <-- ค่าเวลาที่ต้องการ (180ms)
  const int MIN_PULSES_THRESHOLD = 2;          // หมุนน้อยกว่า 3 pulses ถือว่าอาจถูกขัดขวาง

  unsigned long startTime = millis();          // สำหรับ timeout โดยรวม

  bool slowing = false;

  while (true) {
    // Timeout โดยรวม (ป้องกันลูปค้าง)
    if (millis() - startTime > 15000) {
      Serial.println("WARNING: move_fw timeout 15 วินาที");
      break;
    }

    delay(5);

    long currentPulsesL = encoder1.Poss_L();
    long pulsesTraveled = abs(currentPulsesL - startPulsesL);
    float distanceTraveled = pulsesTraveled / pulsesPerCm;
    float remaining = dist - distanceTraveled;

    // === ตรวจจับการชนด้วย encoder ===
    long deltaPulses = abs(currentPulsesL - lastPulsesL);
    if (deltaPulses < MIN_PULSES_THRESHOLD) {
      if (millis() - lastMoveTime > STUCK_TIME_MS) {
        robot.motor('A', -20);
        robot.motor('C', -20);
        delay(30);
        break;
      }
    } else {
      lastMoveTime = millis();   // encoder ยังหมุนปกติ → reset เวลา
    }
    lastPulsesL = currentPulsesL;

    // === โหมดช้าลงเมื่อใกล้ถึงเป้า ===
    if (remaining <= dist * 0.25f && !slowing) {
      slowing = true;
      Serial.printf("เข้าโหมดช้าลง | เหลือ %.1f cm\n", remaining);
    }

    int currentSpeed = sl;
    if (slowing) {
      currentSpeed = sl * 0.57f;
      currentSpeed = constrain(currentSpeed, 18, sl);
    }

    // === หยุดเมื่อถึงระยะทางเป้า ===
    if (remaining <= 1.3f) {
      Serial.println("ถึงระยะทางเป้าแล้ว");
      break;
    }

    // === Gyro PID ===
    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(currentAngle - targetAngle);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt < 0.001f) dt = 0.005f;
    if (dt > 0.1f) dt = 0.02f;

    integral += angleError * dt;
    integral = constrain(integral, -60.0f, 60.0f);

    float derivative = (angleError - lastError) / dt;
    lastError = angleError;

    float correction = Kp * angleError + Ki * integral + Kd * derivative;
    correction = constrain(correction, -40, 40);

    int leftSpeed  = currentSpeed - correction;
    int rightSpeed = currentSpeed + correction;

    leftSpeed  = constrain(leftSpeed,  -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);

    robot.motor('A', leftSpeed);
    robot.motor('C', rightSpeed);
  }

  // === หยุดหุ่นยนต์ + Inertia Brake ===
  robot.motor('A', 0);
  robot.motor('C', 0);
  delay(10);

  int brakePower = (sl > 65) ? -22 : -15;
  robot.motor('A', brakePower);
  robot.motor('C', brakePower);
  delay(30);

  robot.motor('A', 0);
  robot.motor('C', 0);

  // === รายงานผล ===
  long finalPulsesL = encoder1.Poss_L();
  float finalDist = abs(finalPulsesL - startPulsesL) / pulsesPerCm;

  Serial.printf("สิ้นสุด → ได้จริง: %.2f cm (เป้า: %.1f cm) | Error: %.2f cm\n", 
                finalDist, dist, finalDist - dist);
}