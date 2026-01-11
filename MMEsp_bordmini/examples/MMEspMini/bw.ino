void move_bw(int sl, float kp, float dist) {
  extern MMEsp_bordmini board;
  extern EncoderLibrary encoder;

  float targetPulses = dist * pulsesPerCm;
  long startPulsesL = encoder.Poss_L();
  gyro.recalibrateGyro();
  gyro.resetAngles();
  delay(30);
  float initialAngle = averageGyroZ(20);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  Serial.printf("=== move_bw | speed: %d | kp: %.2f | dist: %.1f cm ===\n", sl, kp, dist);

  bool slowing = false;

  while (true) {
    delay(5);  // loop ถี่ขึ้นมาก
    long currentPulses = encoder.Poss_L();
    long pulsesTraveled = abs(currentPulses - startPulsesL);
    float distanceTraveled = pulsesTraveled / pulsesPerCm;

    float remaining = dist - distanceTraveled;

    float slowZone = dist * 0.3f;
    if (remaining <= slowZone && !slowing) {
      slowing = true;
      Serial.println("เข้าโหมดช้าลง 30% (เหลือ 30% ของระยะถอยหลัง)");
    }

    int currentSpeed = -sl;  // ถอยหลัง
    if (slowing) {
      currentSpeed = - (sl * 0.7f);  // ลดลง 30%
      currentSpeed = min(currentSpeed, -15);  // ลดขั้นต่ำเพื่อ PID แก้เอียงดีขึ้น
    }

    if (remaining <= 1.5f) {
      board.Motor(0, 0);
      Serial.println("ถึงเป้าแล้ว (หยุดเรียบร้อย)");
      break;
    }

    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(targetAngle - currentAngle);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt <= 0.001f) dt = 0.02f;
    if (dt > 0.08f) dt = 0.02f;

    integral += angleError * dt;
    integral = constrain(integral, -60, 60);

    float derivative = (angleError - lastError) / dt;
    lastError = angleError;

    float effective_kp = kp;
    float effective_kd = 0.2f;
    if (slowing) {
      effective_kp = kp * 1.5f;      // เพิ่ม Kp ตอนช้า
      effective_kd = 0.35f;          // เพิ่ม Kd เพื่อเบรกเอียง
    }

    float correction = effective_kp * angleError + 0.01f * integral + effective_kd * derivative;

    // เพิ่มช่วง correction ตอนช้าเพื่อไม่เอียง
    correction = constrain(correction, slowing ? -45 : -30, slowing ? 45 : 30);

    int leftSpeed  = currentSpeed + correction;
    int rightSpeed = currentSpeed - correction;

    leftSpeed  = constrain(leftSpeed, -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);

    board.Motor(leftSpeed, rightSpeed);

  }

  board.Motor(10, 10);   // เบรกเบา ๆ ย้อนทิศสำหรับถอยหลัง
  delay(20);
  board.Motor(1, 1);
  delay(50);

  float finalDist = abs(encoder.Poss_L() - startPulsesL) / pulsesPerCm;
  Serial.printf("สิ้นสุด → ได้จริง: %.1f cm (เป้า: %.1f cm) Error: %.1f cm\n", 
                finalDist, dist, finalDist - dist);
}