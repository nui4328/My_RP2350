void setrobot_fw(int _time)
  {
    _set_f = true;
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
    do{robot.motor('A', -25); robot.motor('C', -25);}while(robot.adcRead(1) < robot.adcMD(1)-100);
    robot.motor('A', 5); robot.motor('C', 5);
    delay(25);

    for(int i = 0; i<_time; i++)
      {               
        while(1)
          {
            if(robot.adcRead(1) < robot.adcMD(1) && robot.adcRead(8) > robot.adcMD(8)) 
              {
                robot.motor('A', -5); robot.motor('C', 15);     
              }
            else if(robot.adcRead(1) > robot.adcMD(1) && robot.adcRead(8) < robot.adcMD(8))
              {
                robot.motor('A', 15); robot.motor('C', -5);     
              }
            else if(robot.adcRead(1) < robot.adcMD(1) && robot.adcRead(8) < robot.adcMD(8))
              {   
                robot.motor('A', -10); robot.motor('C', -10);
                delay(30);
                break;            
              }
            else
              {
                  delay(5);
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

                  float correction = 2.5 * angleError + 0.01f * integral + 0.2f * derivative;
                  correction = constrain(correction, -30, 30);

                  int leftSpeed  = 20 - correction;
                  int rightSpeed = 20 + correction;

                  leftSpeed  = constrain(leftSpeed,  -100, 100);
                  rightSpeed = constrain(rightSpeed, -100, 100);

                  robot.motor('A', leftSpeed);
                  robot.motor('C', rightSpeed);
                
              }
          }
        if(i < _time-1)
          {
            while(1)
              {
                robot.motor('A', -15); robot.motor('C', -15);                
                if(robot.adcRead(1) > robot.adcMD(1) && robot.adcRead(8) > robot.adcMD(8))
                  {
                    break;
                  }
              }
          }
      }
   
    robot.motor('A', -1); robot.motor('C', -1); delay(30);  
    robot.motor('A', 0); robot.motor('C', 0); delay(25);
    delay(50);
    gyro.resetAngles();
    ch_line = false;
  }


void setrobot_bw(int _time)
  {
    _set_b = true;
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
    do{robot.motor('A', 20); robot.motor('C', 20);}while(robot.adcRead(0) < robot.adcMD(0)-100);
    robot.motor('A', -5); robot.motor('C', -5);
    delay(25);

    for(int i = 0; i<_time; i++)
      {               
        while(1)
          {
            if(robot.adcRead(9) < robot.adcMD(9) && robot.adcRead(0) > robot.adcMD(0)) 
              {
                robot.motor('A', -25); robot.motor('C', 5);     
              }
            else if(robot.adcRead(9) > robot.adcMD(9) && robot.adcRead(0) < robot.adcMD(0))
              {
                robot.motor('A', 5); robot.motor('C', -25);     
              }
            else if(robot.adcRead(9) < robot.adcMD(9) && robot.adcRead(0) < robot.adcMD(0))
              {   
                robot.motor('A', 5); robot.motor('C', 5);
                delay(30);
                break;            
              }
            else
              {
                  delay(5);
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

                  float correction = 2.5 * angleError + 0.01f * integral + 0.2f * derivative;
                  correction = constrain(correction, -30, 30);

                  int leftSpeed  = -(20 + correction);
                  int rightSpeed = -(20 - correction);

                  leftSpeed  = constrain(leftSpeed,  -100, 100);
                  rightSpeed = constrain(rightSpeed, -100, 100);

                  robot.motor('A', leftSpeed);
                  robot.motor('C', rightSpeed);
                
              }
          }
        if(i < _time-1)
          {
            while(1)
              {
                robot.motor('A', 20); robot.motor('C', 20);                
                if(robot.adcRead(0) > robot.adcMD(0) && robot.adcRead(9) > robot.adcMD(9))
                  {
                    break;
                  }
              }
          }
      }
    robot.motor('A', 1); robot.motor('C', 1); delay(30);  
    robot.motor('A', 0); robot.motor('C', 0); delay(25);
    delay(50);
    gyro.resetAngles();
    ch_line = false;
  }
