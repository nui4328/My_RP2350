

void set_f(int _time)
  {
    _set_f = true;
    do{robot.motor('A', -25); robot.motor('C', -25);}while(robot.adcRead(1) < robot.adcMD(1)-100);
    robot.motor('A', 5); robot.motor('C', 5);
    delay(25);
    for(int i = 0; i<_time; i++)
      {               
        while(1)
          {
            if(robot.adcRead(1) < robot.adcMD(1) && robot.adcRead(2) > robot.adcMD(2)) 
              {
                robot.motor('A', -5); robot.motor('C', 25);     
              }
            else if(robot.adcRead(1) > robot.adcMD(1) && robot.adcRead(2) < robot.adcMD(2))
              {
                robot.motor('A', 25); robot.motor('C', -5);     
              }
            else if(robot.adcRead(0) < robot.adcMD(0) && robot.adcRead(3) < robot.adcMD(3)
                  || robot.adcRead(1) < robot.adcMD(1) && robot.adcRead(2) < robot.adcMD(2))
              {   
                robot.motor('A', -25); robot.motor('C', -25);
                delay(30);
                break;            
              }
            else
              {
                robot.motor('A', 15); robot.motor('C', 15);
                
              }
          }
        if(i < _time-1)
          {
            while(1)
              {
                robot.motor('A', -15); robot.motor('C', -15);                
                if(robot.adcRead(0) > robot.adcMD(0) && robot.adcRead(3) > robot.adcMD(3))
                  {
                    break;
                  }
              }
            while(1)
              {
                robot.motor('A', -15); robot.motor('C', -15); 
                if(robot.adcRead(1) > robot.adcMD(1) || robot.adcRead(2) > robot.adcMD(2))
                  {
                    robot.motor('A', 5); robot.motor('C', 5); 
                    delay(10);
                    break;
                  }
              }
          }
      }
    while(1)
          {
            if(robot.adcRead(0) < robot.adcMD(0)-100 && robot.adcRead(3) > robot.adcMD(3)) 
              {
                do{robot.motor('A', -5); robot.motor('C', 25); }while(robot.adcRead(3) > robot.adcMD(3)+100);
                robot.motor('A', 5); robot.motor('C', -15); delay(30); 
                robot.motor('A', 0); robot.motor('C', 0); delay(25);
                break;     
              }
            else if(robot.adcRead(0) > robot.adcMD(0) && robot.adcRead(3) < robot.adcMD(3)-100)
              {
                do{robot.motor('A', 25); robot.motor('C', -5); }while(robot.adcRead(0) > robot.adcMD(0)+100);
                robot.motor('A', 15); robot.motor('C', -5); delay(30);  
                robot.motor('A', 0); robot.motor('C', 0); delay(25);
                break;         
              }

            else
              {
                robot.motor('A', 15); robot.motor('C', 15); 
              }
          }
    gyro.resetAngles();     
    gyro.recalibrateGyro();
    delay(50);
    gyro.resetAngles();
      ch_line = false;
  }

void set_b(int _time)
  {    
    _set_b = true;
    do{robot.motor('A', 15); robot.motor('C', 15); }while(robot.adcRead(5) < robot.adcMD(5));
    robot.motor('A', -15); robot.motor('C', -15); 
    delay(25);
    for(int i = 0; i<_time; i++)
      {               
        while(1)
          {
            if(robot.adcRead(5) < robot.adcMD(5)-100 && robot.adcRead(6) > robot.adcMD(6)) 
              {
                ch_lrs = 'l';
                
                robot.motor('A', -20); robot.motor('C', 5);         
              }
            else if(robot.adcRead(5) > robot.adcMD(5) && robot.adcRead(6) < robot.adcMD(6)-100)
              {
                ch_lrs = 'r';
                robot.motor('A', 5); robot.motor('C', -20);        
              }
           else if(robot.adcRead(5) < robot.adcMD(5)-50 && robot.adcRead(6) < robot.adcMD(6)-50
                  ||robot.adcRead(5) < robot.adcMD(5)-50 && robot.adcRead(7) < robot.adcMD(7)-50
                  ||robot.adcRead(4) < robot.adcMD(4)-50 && robot.adcRead(7) < robot.adcMD(7)-50
                  ||robot.adcRead(4) < robot.adcMD(4)-50 && robot.adcRead(6) < robot.adcMD(6)-50)
              {   
                robot.motor('A', 25); robot.motor('C', 25); delay(20);
                if(ch_lrs == 'l')
                  {
                    robot.motor('A', 25); robot.motor('C', 10); delay(25);
                  }
                else if(ch_lrs == 'r')
                  {
                    robot.motor('A', 10); robot.motor('C', 25); delay(25);
                  }
                else
                  {
                    robot.motor('A', 25); robot.motor('C', 25); delay(30);
                  }
                robot.motor('A',5); robot.motor('C', 5); delay(50);
                break;            
              }
            else
              {
                robot.motor('A', -15); robot.motor('C', -15); 
              }
          }
        if(i < _time-1)
          {
            
            while(1)
              {
                robot.motor('A', 15); robot.motor('C', 15); 
                if(robot.adcRead(5) > robot.adcMD(5) && robot.adcRead(6) > robot.adcMD(6))
                  {
                    break;
                  }
              }
            
            while(1)
              {
                robot.motor('A', 15); robot.motor('C', 15); 
                if(robot.adcRead(7) > robot.adcMD(7) || robot.adcRead(4) > robot.adcMD(4))
                  {
                    robot.motor('A', -15); robot.motor('C', -15); delay(10);
                    break;
                  }
              }
          }
          
      }

    while(1)
          {
            if(robot.adcRead(4) < robot.adcMD(4)-100 && robot.adcRead(7) > robot.adcMD(7)) 
              {
                do{robot.motor('A', -25); robot.motor('C', 5); }while(robot.adcRead(7) > robot.adcMD(7)+100);
                robot.motor('A', 15); robot.motor('C', -5); delay(25); 
                robot.motor('A', 0); robot.motor('C', 0); delay(25);
                break;     
              }
            else if(robot.adcRead(4) > robot.adcMD(4) && robot.adcRead(7) < robot.adcMD(7)-100)
              {
                do{robot.motor('A', 5); robot.motor('C', -25); }while(robot.adcRead(4) > robot.adcMD(4)-100);
                robot.motor('A', -5); robot.motor('C', 15); delay(25);  
                robot.motor('A', 0); robot.motor('C', 0); delay(25);
                break;         
              }
       
            else
              {
                robot.motor('A', -15); robot.motor('C', -15); 
              }
          }
    gyro.resetAngles();     
    gyro.recalibrateGyro();
    delay(50);
    gyro.resetAngles();
    ch_line = false;
      
  }