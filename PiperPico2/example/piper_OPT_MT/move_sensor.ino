
int untrasonic()
  {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distanceCm = duration * 0.034 / 2;
    delay(60);
    return duration * 0.034 / 2;
  }
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
    if (abs(currentADC) >= targetADC) {  // ปรับ 8 ได้ตามความเสถียรของเซ็นเซอร์
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

    int leftSpeed = sl - correction;
    int rightSpeed = sl + correction;

    leftSpeed = constrain(leftSpeed, -100, 100);
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
  // gyro.resetAngles();
  // delay(30);
  float initialAngle = averageGyroZ(10);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;          
  unsigned long lastTime = millis();

  Serial.printf("=== move_adc | speed: %d | kp: %.2f | targetADC: %d ===\n",
                sl, kp, targetADC);

  while (true) {
    delay(5);

    int currentADC3 = robot.adcRead(3);
    int currentADC4 = robot.adcRead(4);

    if (currentADC3 >= targetADC || currentADC4 >= targetADC) {
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

    int leftSpeed = sl - correction;
    int rightSpeed = sl + correction;

    leftSpeed = constrain(leftSpeed, -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);

    robot.motor('A', leftSpeed);
    robot.motor('C', rightSpeed);
    if (robot.adcRead(5) > 1500) {
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

  int finalADC3 = robot.adcRead(3);
  int finalADC4 = robot.adcRead(4);

  Serial.printf("สิ้นสุด move_adc → ADC3: %d | ADC4: %d\n",
                finalADC3, finalADC4);
}


void move_untrasonic(int sl, float kp, int targetADC) {
  extern PiperPico2 robot;
  extern EncoderLibrarys encoder;  // ยังคงไว้เผื่อใช้ในอนาคต

  // ===== เริ่มต้น =====
  // gyro.resetAngles();
  // delay(30);
  
  float initialAngle = averageGyroZ(10);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;          
  unsigned long lastTime = millis();

  Serial.printf("=== move_adc | speed: %d | kp: %.2f | targetADC: %d ===\n",
                sl, kp, targetADC);

  while (true) {
    delay(60);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distanceCm = duration * 0.034 / 2;

    if (distanceCm <= targetADC ) {
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

    int leftSpeed = sl - correction;
    int rightSpeed = sl + correction;

    leftSpeed = constrain(leftSpeed, -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);

    robot.motor('A', leftSpeed);
    robot.motor('C', rightSpeed);
  }

  // ===== หยุดด้วย brake เบา ๆ เพื่อลด inertia =====
  robot.motor('A', -10);
  robot.motor('C', -10);
  delay(25);
  robot.motor('A', -1);
  robot.motor('C', -1);
  delay(60);

}

void move_stuck(int sl, float kp, float dist) {
  extern PiperPico2 robot;

  if (dist <= 0.0f) return;
  if (sl <= 0) sl = 40;

  // เตรียมระบบก่อนเริ่มเคลื่อนที่
  gyro.resetAngles();
  delay(50);
  
  float targetAngle = averageGyroZ(20);   // มุมเป้าหมาย (เดินตรง)

  // PID Control
  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  const float Kp = kp;
  const float Ki = 0.09f;   // สามารถปรับหรือรับเป็นพารามิเตอร์ได้
  const float Kd = 0.13f;

  Serial.printf("=== move_stuck | speed: %d | Kp: %.2f | dist: %.1f cm | (มีระบบตรวจติดขัด) ===\n",
                sl, Kp, dist);

  // ตัวแปรสำหรับคำนวณระยะทางและตรวจติดขัด
  long startPulsesL   = encoder1.Poss_L();
  long lastPulsesL    = startPulsesL;
  unsigned long lastMoveTime = millis();
  unsigned long startTime    = millis();

  const unsigned long STUCK_TIME_MS     = 160;   // ปรับจาก 140 เป็น 160 (เสถียรขึ้น)
  const int MIN_PULSES_THRESHOLD        = 2;

  bool slowing = false;

  while (true) {
    // Timeout ป้องกันลูปค้าง
    if (millis() - startTime > 15000) {
      Serial.println("WARNING: move_stuck timeout 15 วินาที");
      break;
    }

    delay(5);

    long currentPulsesL = encoder1.Poss_L();
    float distanceTraveled = abs(currentPulsesL - startPulsesL) / pulsesPerCm;
    float remaining = dist - distanceTraveled;

    // === ระบบตรวจจับการติดขัด (Stuck Detection) ===
    long deltaPulses = abs(currentPulsesL - lastPulsesL);
    if (deltaPulses < MIN_PULSES_THRESHOLD) {
      if (millis() - lastMoveTime > STUCK_TIME_MS) {
        Serial.println("DETECTED: หุ่นยนต์ติดขัด → ถอยหลังเล็กน้อย");
        robot.motor('A', -20);
        robot.motor('C', -20);
        delay(35);
        break;
      }
    } else {
      lastMoveTime = millis();   // encoder ยังหมุนปกติ
    }
    lastPulsesL = currentPulsesL;

    // === โหมดช้าลงเมื่อใกล้ถึงเป้า ===
    if (remaining <= dist * 0.25f && !slowing) {
      slowing = true;
      Serial.printf("เข้าโหมดช้าลง | เหลือ %.1f cm\n", remaining);
    }

    int currentSpeed = slowing ? constrain((int)(sl * 0.57f), 18, sl) : sl;

    // === หยุดเมื่อถึงระยะทางเป้า ===
    if (remaining <= 1.3f) {
      Serial.println("ถึงระยะทางเป้าแล้ว");
      break;
    }

    // === Gyro PID ควบคุมทิศทาง ===
    float currentAngle = gyro.gyro('z');
    float angleError = constrainAngle180(currentAngle - targetAngle);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    if (dt < 0.001f) dt = 0.005f;
    if (dt > 0.1f)   dt = 0.02f;

    integral += angleError * dt;
    integral = constrain(integral, -60.0f, 60.0f);

    float derivative = (angleError - lastError) / dt;
    lastError = angleError;

    float correction = Kp * angleError + Ki * integral + Kd * derivative;
    correction = constrain(correction, -40, 40);

    int leftSpeed  = constrain(currentSpeed - correction, -100, 100);
    int rightSpeed = constrain(currentSpeed + correction, -100, 100);

    robot.motor('A', leftSpeed);
    robot.motor('C', rightSpeed);
  }

  // === หยุดมอเตอร์ + Inertia Brake ===
  robot.motor('A', 0);
  robot.motor('C', 0);
  delay(10);

  int brakePower = (sl > 65) ? -24 : -16;
  robot.motor('A', brakePower);
  robot.motor('C', brakePower);
  delay(35);

  robot.motor('A', 0);
  robot.motor('C', 0);

  // === รายงานผลการทำงาน ===
  long finalPulsesL = encoder1.Poss_L();
  float finalDist = abs(finalPulsesL - startPulsesL) / pulsesPerCm;

  Serial.printf("สิ้นสุด → ได้จริง: %.2f cm (เป้า: %.1f cm) | Error: %.2f cm\n",
                finalDist, dist, finalDist - dist);
}