void setrobot_fw(int _time)
  {
    _set_f = true;
    extern PiperPico2 robot;
    extern EncoderLibrarys encoder;  // ยังคงไว้เผื่อใช้ในอนาคต
    float initialAngle ;
    float targetAngle ;

    // ===== เริ่มต้น =====
     if(_offset == true)
      {
        gyro.resetAngles();
        delay(30);
        initialAngle = averageGyroZ(10);
        targetAngle = initialAngle;
      }
    else  
      {
        initialAngle = averageGyroZ(2);
        targetAngle = initialAngle;
      }

    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();
    while(1)
      {
        robot.motor('A', -25); robot.motor('C', -25);
        if(robot.adcRead(1) > robot.adcMD(1)-100 && robot.adcRead(8) > robot.adcMD(8)-100)
          {
            break;
          }
      }
    robot.motor('A', 0); robot.motor('C', 0);
    delay(15);

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

                  float correction = 1.25 * angleError +  0.02f * derivative;
                  correction = constrain(correction, -30, 30);

                  int leftSpeed  = 20 - correction;
                  int rightSpeed = 20 + correction;

                  leftSpeed  = constrain(leftSpeed,  -100, 100);
                  rightSpeed = constrain(rightSpeed, -100, 100);

                  robot.motor('A', leftSpeed);
                  robot.motor('C', rightSpeed);
            if(robot.adcRead(1) < (robot.adcMD(1) + robot.adcMin(1)) / 2 && robot.adcRead(8) > robot.adcMD(8)) 
              {
                do{robot.motor('A', -5); robot.motor('C', 25);delay(3); } while(robot.adcRead(8) > robot.adcMD(8) );   
                 break;
                    
              }
            else if(robot.adcRead(1) > robot.adcMD(1) && robot.adcRead(8) < (robot.adcMD(8) + robot.adcMin(8)) / 2)
              {
                do{robot.motor('A', 25); robot.motor('C', -5);delay(3); } while(robot.adcRead(1) > robot.adcMD(1) );   
                 break; 
              }
            else if(robot.adcRead(1) < robot.adcMD(1) && robot.adcRead(8) <robot.adcMD(8))
              {   
                robot.motor('A', -20); robot.motor('C', -20);
                delay(30);
                break;            
              }
           
          }
        if(i < _time-1)
          {
            robot.motor('A', -15); robot.motor('C', -15);
            delay(20);
            while(1)
              {
                robot.motor('A', -15); robot.motor('C', -15);                
                if(robot.adcRead(1) > (robot.adcMD(1) + robot.adcMin(1)) / 2 || robot.adcRead(8) > (robot.adcMD(1) + robot.adcMin(8)) / 2)
                  {
                    robot.motor('A', 5); robot.motor('C',5); delay(10);  
                    robot.motor('A', 0); robot.motor('C', 0); delay(10);
                    
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

    float initialAngle ;
    float targetAngle ;

    // ===== เริ่มต้น =====
     if(_offset == true)
      {
        gyro.resetAngles();
        delay(30);
        initialAngle = averageGyroZ(10);
        targetAngle = initialAngle;
      }
    else  
      {
        initialAngle = averageGyroZ(2);
        targetAngle = initialAngle;
      }
    

    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();
    do{robot.motor('A', 20); robot.motor('C', 20);}while(robot.adcRead(0) < robot.adcMD(0)-100 || robot.adcRead(9) < robot.adcMD(9)-100);
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

                  float correction = 0.35 * angleError +  0.02f * derivative;
                  correction = constrain(correction, -30, 30);

                  int leftSpeed  = -(20 + correction);
                  int rightSpeed = -(20 - correction);

                  leftSpeed  = constrain(leftSpeed,  -100, 100);
                  rightSpeed = constrain(rightSpeed, -100, 100);

                  robot.motor('A', leftSpeed);
                  robot.motor('C', rightSpeed);
            if(robot.adcRead(9) < robot.adcMD(9)-100  && robot.adcRead(0) > robot.adcMD(0)) 
              { 
                
                do{robot.motor('A', -30); robot.motor('C', 5);delay(3); } while(robot.adcRead(0) > (robot.adcMD(0) + robot.adcMin(0)) / 2);   
                 break; 
              }
            else if(robot.adcRead(9) > robot.adcMD(9) && robot.adcRead(0) < robot.adcMD(0)-100 )
              {
                
                do{robot.motor('A', 5); robot.motor('C', -30); delay(3);} while(robot.adcRead(9) > (robot.adcMD(9) + robot.adcMin(9)) / 2);   
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
            robot.motor('A', 15); robot.motor('C', 15);
            delay(20);
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
            
          }
      }
    robot.motor('A', 0); robot.motor('C', 0); delay(15);
    delay(50);
    gyro.resetAngles();
    ch_line = false;
  }
