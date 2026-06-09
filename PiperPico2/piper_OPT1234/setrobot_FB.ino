void setrobot_fw(int _time)
  {
    _set_f = true;
    extern PiperPico2 robot;
    extern EncoderLibrarys encoder;  // ยังคงไว้เผื่อใช้ในอนาคต

    // ===== เริ่มต้น =====
    // gyro.resetAngles();
    // delay(30);
    // float initialAngle = averageGyroZ(2);
    float targetAngle = gyro.gyro('z');

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
                robot.motor('A', -5); robot.motor('C', 25);     
              }
            else if(robot.adcRead(1) > robot.adcMD(1) && robot.adcRead(8) < robot.adcMD(8))
              {
                robot.motor('A', 25); robot.motor('C', -5);     
              }
            else if(robot.adcRead(1) < (robot.adcMD(1) + robot.adcMin(1)) / 2 && robot.adcRead(8) < (robot.adcMD(8) + robot.adcMin(8)) / 2)
              {   
                robot.motor('A', -20); robot.motor('C', -20);
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

                  float correction = 0.5 * angleError + 0.001f * integral + 0.02f * derivative;
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
   
    robot.motor('A', 5); robot.motor('C', 5); delay(30);  
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

                  float correction = 0.5 * angleError + 0.01f * integral + 0.02f * derivative;
                  correction = constrain(correction, -30, 30);

                  int leftSpeed  = -(25 + correction);
                  int rightSpeed = -(25 - correction);

                  leftSpeed  = constrain(leftSpeed,  -100, 100);
                  rightSpeed = constrain(rightSpeed, -100, 100);

                  robot.motor('A', leftSpeed);
                  robot.motor('C', rightSpeed);
            if(robot.adcRead(9) < robot.adcMD(9)  && robot.adcRead(0) > robot.adcMD(0)) 
              { 
                robot.motor('A', 1); robot.motor('C', 10);
                delay(10);
                do{robot.motor('A', -20); robot.motor('C', 3);delay(3); } while(robot.adcRead(0) > (robot.adcMD(0) + robot.adcMin(0)) / 2);   
                 break; 
              }
            else if(robot.adcRead(9) > robot.adcMD(9) && robot.adcRead(0) < robot.adcMD(0) )
              {
                robot.motor('A', 10); robot.motor('C', 1);
                delay(10);
                do{robot.motor('A', 3); robot.motor('C', -20); delay(3);} while(robot.adcRead(9) > (robot.adcMD(9) + robot.adcMin(9)) / 2);   
                 break;    
              }
            else if(robot.adcRead(9) < (robot.adcMD(9) + robot.adcMin(9)) / 2 && robot.adcRead(0) < (robot.adcMD(0) + robot.adcMin(0)) / 2)
              {   
                robot.motor('A', 15); robot.motor('C', 15);
                delay(20);
                break;            
              }
         
          }
        if(i < _time-1)
          {
            while(1)
              {
                robot.motor('A', 15); robot.motor('C', 15);
                if(robot.adcRead(0) > robot.adcMD(0) || robot.adcRead(9) > robot.adcMD(9))
                  {
                    robot.motor('A', -5); robot.motor('C',-5); delay(10);  
                    robot.motor('A', 0); robot.motor('C', 0); delay(10);
                    
                      break;
                  }
              }
            while(1)
              {
                robot.motor('A', 20); robot.motor('C', 20);                
                if(robot.adcRead(0) > robot.adcMD(0) || robot.adcRead(9) > robot.adcMD(9))
                  {
                    robot.motor('A', -5); robot.motor('C',-5); delay(30);  
                    robot.motor('A', 0); robot.motor('C', 0); delay(25);
                    
                    break;
                  }
              }
          }
      }
    robot.motor('A', 0); robot.motor('C', 0); delay(15);
    delay(50);
    gyro.resetAngles();
    ch_line = false;
  }
