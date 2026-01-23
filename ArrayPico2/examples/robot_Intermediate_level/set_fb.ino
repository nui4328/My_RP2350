

void set_f(int _time)
  {
    _set_f = true;
    do{robot.Motor(-10, -10);}while(robot.adcRead(1) < robot.adcMD(1));
    delay(20);
    for(int i = 0; i<_time; i++)
      {               
        while(1)
          {
            if(robot.adcRead(1) < robot.adcMD(1) && robot.adcRead(2) > robot.adcMD(2)) 
              {
                 robot.Motor(-2, 20);        
              }
            else if(robot.adcRead(1) > robot.adcMD(1) && robot.adcRead(2) < robot.adcMD(2))
              {
                robot.Motor(20, -2);           
              }
            else if(robot.adcRead(0) < robot.adcMD(0) && robot.adcRead(3) < robot.adcMD(3)
                  || robot.adcRead(1) < robot.adcMD(1) && robot.adcRead(2) < robot.adcMD(2))
              {   
                robot.Motor(-20, -20);delay(20);
                robot.Motor(1, 1);delay(10);
                //robot.Motor(0, 0); delay(10);
                break;            
              }
            else
              {
                robot.Motor(10, 10); 
              }
          }
        if(i < _time-1)
          {
            while(1)
              {
                robot.Motor(-15, -15);
                if(robot.adcRead(1) > robot.adcMD(1) && robot.adcRead(2) > robot.adcMD(2))
                  {
                    break;
                  }
              }
            while(1)
              {
                robot.Motor(-15, -15);
                if(robot.adcRead(0) > robot.adcMD(0) || robot.adcRead(3) > robot.adcMD(3))
                  {
                    robot.Motor(5, 5); delay(10);
                    break;
                  }
              }
          }
      }
      robot.Motor(0, 0); delay(50);
      ch_line = false;
  }

void set_b(int _time)
  {    
    _set_b = true;
    do{robot.Motor(10, 10);}while(robot.adcRead(5) < robot.adcMD(5));
    delay(20);
    for(int i = 0; i<_time; i++)
      {               
        while(1)
          {
            if(robot.adcRead(5) < robot.adcMD(5)-50 && robot.adcRead(6) > robot.adcMD(6)) 
              {
                ch_lrs = 'l';
                 robot.Motor(-15, 5);        
              }
            else if(robot.adcRead(5) > robot.adcMD(5) && robot.adcRead(6) < robot.adcMD(6)-50)
              {
                ch_lrs = 'r';
                robot.Motor(5, -15);           
              }
           else if(robot.adcRead(5) < robot.adcMD(5)-50 && robot.adcRead(6) < robot.adcMD(6)-50)
              {   
                if(ch_lrs == 'l')
                  {
                     robot.Motor(25, 15);delay(20);
                  }
                else if(ch_lrs == 'r')
                  {
                     robot.Motor(15, 25);delay(20);
                  }
                else
                  {
                     robot.Motor(35, 35);delay(30);
                  }
                robot.Motor(1, 1); delay(50);
                break;            
              }
            else
              {
                robot.Motor(-10, -10);
              }
          }
        if(i < _time-1)
          {
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
          }
      }
     gyro.resetAngles();
    ch_line = false;
      
  }