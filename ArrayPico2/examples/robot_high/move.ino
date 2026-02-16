void Move_forward (int spl, int spr, float kps, int targetDistanceCm) 
{ 
  extern ArrayPico2 robot;
  extern EncoderLibrarys encoder;
  extern my_BMI160 gyro;
  encoder.resetEncoders();  
    
  float targetPulses = targetDistanceCm * pulsesPerCm;

  robot.Motor(-1, -1); delay(10);
  gyro.resetAngles();

  float yaw_offset = gyro.gyro('z');
  float _integral = 0;
  float _prevErr = 0;
  float prevT = millis();    

    // === พารามิเตอร์ Ramp-up ===
    const int startSpeed = 20;              // ความเร็วเริ่มต้น (ต่ำเพื่อออกตัวนุ่ม)
    const float rampUpPulses = targetPulses * 0.4;  // เร่งใน 20% แรกของระยะทาง
    int currentMaxSpeed = startSpeed;       // ความเร็วสูงสุดปัจจุบัน (จะเพิ่มขึ้นเรื่อย ๆ)

    while (true) 
    {
        float rightPulses = encoder.Poss_R();
        float currentPulses = rightPulses;

        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001;
        prevT = now;

        float yaw = gyro.gyro('z') - yaw_offset;
        float err = yaw;

        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float corr = kps * err + 0.00001 * _integral + 0.035 * deriv;

        // === Ramp-up: ค่อย ๆ เพิ่มความเร็วสูงสุด ===
        if (currentPulses < rampUpPulses) {
            float rampFactor = currentPulses / rampUpPulses;
            currentMaxSpeed = startSpeed + (int)((spl - startSpeed) * rampFactor);
            if (currentMaxSpeed > spl) currentMaxSpeed = spl;
        } else {
            currentMaxSpeed = spl;  // หลัง 20% แรก → เต็มสปีด
        }

        // ความเร็วซ้าย-ขวา แยกกัน (chopsticks style)
        int baseLeftSpeed = currentMaxSpeed;
        int baseRightSpeed = (spr * currentMaxSpeed) / spl;  // รักษาอัตราส่วน spr/spl

        int leftSpeed = constrain(baseLeftSpeed - (int)corr, -100, 100);
        int rightSpeed = constrain(baseRightSpeed + (int)corr, -100, 100);

        robot.Motor(leftSpeed, rightSpeed);        
        // === หยุดเมื่อถึงเป้า ===
        if (currentPulses >= targetPulses) 
            {
                robot.Motor(-2, -5);
                delay(100);
                robot.Motor(0, 0);
                delay(100);
                turnGyro(90, 90);      // หมุนขวา 90°
                sett_f = false;
                set_bb = false;
                break;
            }
        
         
        // === ตรวจสี ===
        
        if ( tcs0.readFast() ) 
           {
            String col = tcs0.getColor();
            //Serial.println(col);
            
            if (col == "RED")
                {
                    robot.Motor(-spl, -spr);
                    delay(30);
                    robot.playTone(3000, 100);delay(100); 
                    robot.Motor(-1, -1);
                    delay(100);
                    do{ robot.Motor(-20, -20);} while(robot.adcRead(1) < robot.adcMD(1)-30);
                    robot.Motor(10, 10);
                    delay(20);
                    set_f(1);
                    
                    servo_red();
                    set_f(2);
                    move_bw(60, 1.5, bw_distance);
                    turnGyro(90, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                    _set_f = false;
                    _set_b = false;
                    break;

                }
            else if (col == "GREEN")
                {
                    robot.Motor(-spl, -spr);
                    delay(30);
                    robot.playTone(3000, 100);delay(100); 
                    robot.playTone(3000, 100);delay(100); 
                    robot.Motor(-1, -1);
                    delay(100);
                    do{ robot.Motor(-20, -20);} while(robot.adcRead(1) < robot.adcMD(1)-30);
                    robot.Motor(10, 10);
                    delay(20);
                    set_f(1);
                    
                    servo_green();
                    set_f(2);
                    move_bw(60, 1.5, bw_distance);
                    turnGyro(90, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                    _set_f = false;
                    _set_b = false;
                    
                    break;

                }
            else if (col == "BLUE")
                {
                    robot.Motor(-spl, -spr);
                    delay(30);
                    robot.playTone(3000, 100);delay(100); 
                    robot.playTone(3000, 100);delay(100); 
                    robot.playTone(3000, 100);delay(100); 
                    robot.Motor(-1, -1);
                    delay(100);
                    do{ robot.Motor(-20, -20);} while(robot.adcRead(1) < robot.adcMD(1)-30);
                    robot.Motor(10, 10);
                    delay(20);
                    set_f(1);
                    
                    servo_blue();
                    set_f(2);
                    move_bw(60, 1.5, bw_distance);
                    turnGyro(90, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                    _set_f = false;
                    _set_b = false;
                    break;

                }
             else if (col == "YELLOW")
                {
                    robot.Motor(-spl, -spr);
                    delay(20);
                    robot.Motor(-2, -2);
                    delay(100);
                    robot.playTone(3000, 100);delay(100); 
                    robot.playTone(3000, 100);delay(100); 
                    robot.playTone(3000, 100);delay(100); 
                    robot.playTone(3000, 100);delay(100);
                    robot.Motor(-1, -1);
                    delay(100);

                    // เดินหน้าเข้าไปให้เซ็นเซอร์อยู่กลางพื้นเหลืองชัด ๆ (ปรับตามระยะจริงของหุ่นคุณ)
                    move_fw(30, 1.5, 7);
                    col = "NONE";
                    robot.Motor(-1, -1);
                    delay(200);  // รอให้หุ่นนิ่ง เซ็นเซอร์อ่านค่าคงที่

                    // === Confirm สีเหลืองด้วยการตรวจ ratio ดิบโดยตรง (ไม่พึ่ง getColor()) ===
                    int yellow_count = 0;
                    for (int i = 0; i < 8; i++) 
                      {
                        if (tcs0.readFast() ) 
                        {
                            if (tcs0.getColor() == "YELLOW") 
                            {
                                yellow_count++;
                                robot.playTone(3000, 100);delay(100); 
                            }
                        }
                        delay(40);
                     }
                      
                    // === ตัดสินใจ ===
                  if (tcs0.readFast() ) 
                        {
                    if (yellow_count >= 2 || tcs0.getColor() == "YELLOW")  // เจออย่างน้อย 5/10 ครั้ง → ถือว่าเป็นพื้นเหลืองจริง
                    {
                        robot.playTone(3000, 100);delay(100); 
                        robot.playTone(3000, 100);delay(100); 

                        move_bw(60, 1.5, 10);
                        robot.Motor(2, 2);
                        delay(100);
                        servo_yellow();  // ปล่อยแขนเก็บกล่อง

                        move_bw(60, 1.5, bw_distance+4);
                        turnGyro(90, -90);  // หมุนซ้าย 90° (ปรับทิศตาม layout สนามจริง)

                        yello_box = 1;    // สำคัญ: ตั้ง flag ว่ากเก็บเหลืองแล้ว
                        _set_f = false;
                        _set_b = false;

                       
                      }
                      else
                      {
                          // ไม่ใช่เหลืองจริง → แค่แตะขอบ → หมุนเลี่ยง
                          robot.playTone(3000, 200);delay(100); 
                          turnGyro(90, 90);   // หมุนขวา 90°
                          ch_poit++;
                          _set_f = false;
                          _set_b = false; 
                          // เช็คว่าครบ 4 สี + แต้มหรือยัง
                          if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
                              {
                                robot.ServoWrite(16, servo16+90);
                                robot.playTone(3000, 100);delay(100); 
                                robot.playTone(3000, 100);delay(100); 
                                robot.run(); // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN)     
                              }
                              delay(200);
                      }
                    }

                    break;
                }
                   
          
           }
        
        // === ตรวจเส้นหน้า === robot.adcRead  robot.adcMD
        
        if (robot.adcRead(1) < robot.adcMD(1)+100 || robot.adcRead(2) < robot.adcMD(2)+100)
          {          
                    
            robot.Motor(-spl, -spr);
            delay(30);
            robot.Motor(-1, -1);
            delay(200);
            
            while(1)
                {
                   robot.Motor(-20, -20); 
                   if (robot.adcRead(1) > robot.adcMD(1)+100 && robot.adcRead(2) > robot.adcMD(2)+100)
                    {
                        delay(60);
                        break;
                    }
                }
            
            set_f(2);
            do{ robot.Motor(20, 20);} while(robot.adcRead(2) > robot.adcMD(2)-30);
            robot.Motor(-20, -20);
            delay(30);
            robot.Motor(-1, -1);
            delay(500);

              
            if ( tcs0.readFast() ) 
                {
                    String col = tcs0.getColor();
                    //Serial.println(col);
                    if (col == "RED")
                        {
                            do{ robot.Motor(-20, -20);} while(robot.adcRead(1) < robot.adcMD(1)-30);
                            robot.Motor(20, 20);
                            delay(20);
                            set_f(1);
                            
                            servo_red();
                            set_f(2);
                            move_bw(60, 1.5, bw_distance);
                            turnGyro(90, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                            _set_f = false;
                            _set_b = false;
                            
                            break;

                        }
                    else if (col == "GREEN")
                        {
                            do{ robot.Motor(-20, -20);} while(robot.adcRead(1) < robot.adcMD(1)-30);
                            robot.Motor(20, 20);
                            delay(20);
                            set_f(1);
                            
                            servo_green();
                            set_f(2);
                            move_bw(60, 1.5, bw_distance);
                            turnGyro(90, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                            _set_f = false;
                            _set_b = false;                            
                            break;

                            }
                    else if (col == "BLUE")
                        {
                            do{ robot.Motor(-20, -20);} while(robot.adcRead(1) < robot.adcMD(1)-30);
                            robot.Motor(20, 20);
                            delay(20);
                            set_f(1);                            
                            servo_blue();
                            set_f(2);
                            move_bw(60, 1.5, bw_distance);
                            turnGyro(90, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                            _set_f = false;
                            _set_b = false;
                                break;

                            }
                }
             
            turnGyro(90, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
            _set_f = false;
            _set_b = false;
            break;
          
          } 
        // ตรวจเส้นข้าง + เส้นหน้า (ใช้ const ที่ประกาศด้านบน)
        if (robot.adcRead(0) < robot.adcMD(0) - 50 && 
            robot.adcRead(3) > robot.adcMD(3)) {
        robot.Motor(leftSpeed*1.5, rightSpeed / 3);
        gyro.resetAngles();
        Serial.println(">>> เจอเส้นข้างซ้าย → ปรับเลี้ยวขวา");
        } 
        else if (robot.adcRead(0) > robot.adcMD(0) && 
                robot.adcRead(3) < robot.adcMD(3) - 50) {
        robot.Motor(leftSpeed / 3, rightSpeed*1.5);
        gyro.resetAngles();
        Serial.println(">>> เจอเส้นข้างขวา → ปรับเลี้ยวซ้าย");
        } 
    }

    robot.Motor(-1, -1);
    delay(10);
}


void Move_forward_begin (int spl, int spr, float kps, int targetDistanceCm) 
{ 
  extern ArrayPico2 robot;
  extern EncoderLibrarys encoder;
  extern my_BMI160 gyro;
  encoder.resetEncoders();  
    
  float targetPulses = targetDistanceCm * pulsesPerCm;

  robot.Motor(-1, -1); delay(10);
  gyro.resetAngles();

  float yaw_offset = gyro.gyro('z');
  float _integral = 0;
  float _prevErr = 0;
  float prevT = millis();    

    // === พารามิเตอร์ Ramp-up ===
    const int startSpeed = 20;              // ความเร็วเริ่มต้น (ต่ำเพื่อออกตัวนุ่ม)
    const float rampUpPulses = targetPulses * 0.4;  // เร่งใน 20% แรกของระยะทาง
    int currentMaxSpeed = startSpeed;       // ความเร็วสูงสุดปัจจุบัน (จะเพิ่มขึ้นเรื่อย ๆ)

    while (true) 
    {
        float rightPulses = encoder.Poss_R();
        float currentPulses = rightPulses;

        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001;
        prevT = now;

        float yaw = gyro.gyro('z') - yaw_offset;
        float err = yaw;

        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float corr = kps * err + 0.00001 * _integral + 0.035 * deriv;

        // === Ramp-up: ค่อย ๆ เพิ่มความเร็วสูงสุด ===
        if (currentPulses < rampUpPulses) {
            float rampFactor = currentPulses / rampUpPulses;
            currentMaxSpeed = startSpeed + (int)((spl - startSpeed) * rampFactor);
            if (currentMaxSpeed > spl) currentMaxSpeed = spl;
        } else {
            currentMaxSpeed = spl;  // หลัง 20% แรก → เต็มสปีด
        }

        // ความเร็วซ้าย-ขวา แยกกัน (chopsticks style)
        int baseLeftSpeed = currentMaxSpeed;
        int baseRightSpeed = (spr * currentMaxSpeed) / spl;  // รักษาอัตราส่วน spr/spl

        int leftSpeed = constrain(baseLeftSpeed - (int)corr, -100, 100);
        int rightSpeed = constrain(baseRightSpeed + (int)corr, -100, 100);

        robot.Motor(leftSpeed, rightSpeed);        
        // === หยุดเมื่อถึงเป้า ===
        if (currentPulses >= targetPulses) 
            {
                robot.Motor(-2, -5);
                delay(100);
                robot.Motor(0, 0);
                delay(100);
                turnGyro(90, 90);      // หมุนขวา 90°
                sett_f = false;
                set_bb = false;
                break;
            }
        
         
        
    }

    robot.Motor(-1, -1);
    delay(10);
}


void move_fw(int sl, float kp, float dist) {
  extern ArrayPico2 robot;
  extern EncoderLibrarys encoder;
  extern my_BMI160 gyro;
  encoder.resetEncoders();  

  float targetPulses = dist * pulsesPerCm;
  long startPulsesL = encoder.Poss_R();

  gyro.resetAngles();
  delay(30);
  float initialAngle = averageGyroZ(10);
  float targetAngle = initialAngle;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  Serial.printf("=== move_fw | speed: %d | kp: %.2f | dist: %.1f cm ===\n", sl, kp, dist);

  bool slowing = false;

  while (true) {
    delay(5);  // loop ถี่ขึ้นมาก
    long currentPulses = encoder.Poss_R();
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
      robot.Motor(0, 0);
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

    int leftSpeed  = currentSpeed - correction;
    int rightSpeed = currentSpeed + correction;

    leftSpeed  = constrain(leftSpeed, -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);
    robot.Motor(leftSpeed, rightSpeed);
    

  }

  // รอ inertia หยุดสนิท (ตามที่คุณใช้)
  robot.Motor(-15, -15);
  delay(20);
  robot.Motor(-1, -1);
  delay(30);

  float finalDist = abs(encoder.Poss_R() - startPulsesL) / pulsesPerCm;
  Serial.printf("สิ้นสุด → ได้จริง: %.1f cm (เป้า: %.1f cm) Error: %.1f cm\n", 
                finalDist, dist, finalDist - dist);
}