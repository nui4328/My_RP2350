
void arm_ready()       // เตียมมือจับตอนเข้าคีบ
  {
    arm_updown(0);
    arm_shoulder('L', arm_ready_L);  // ค่าเดิมคือ65
    arm_shoulder('R', arm_ready_R);

    arm_hand('L', 75);
    arm_hand('R', 75);
  }

void arm_start_new()
  {
    arm_shoulder('L', arm_ready_L);  // ค่าเดิมคือ65
    arm_shoulder('R', arm_ready_R); 

    arm_updown(4);
    delay(100);
    for(int i = 40; i < 57; i++)
      {
         arm_hand('L', i);  
         arm_hand('R', i);
         delay(10);
      }
    do{robot.motor('D', -15); delay(5);} while(robot.adcRead(6) < 1500);
    robot.motor('D', 5);
    delay(10);
    robot.motor('D', 0);
    delay(300);
    for(int i = 40; i < 57; i++)
      {
         arm_hand('L', i);  
         arm_hand('R', i);
         delay(15);
      }
   
    delay(100);
    arm_hand('L', 40);  
    arm_hand('R', 40);
    delay(200);

  }

  void arm_start_new_L()
  {
    arm_shoulder('L', arm_ready_L);  // ค่าเดิมคือ65
    arm_shoulder('R', arm_ready_R); 
    arm_updown(4);
    delay(200);
    for(int i = 40; i < 57; i++)
      {
         arm_hand('L', i);  
         delay(10);
      }
    arm_updown(1);
    delay(200);
    for(int i = 40; i < 57; i++)
      {
         arm_hand('L', i);  
         delay(10);
      }
   
    delay(200);
    
    do{robot.motor('D', -15); delay(5);} while(robot.adcRead(6) < 1500);
    robot.motor('D', 5);
    delay(10);
    robot.motor('D', 0);
    delay(300);
    arm_hand('L', 40);  
    delay(300);

  }
   void arm_start_new_R()
  {
    arm_shoulder('L', arm_ready_L);  // ค่าเดิมคือ65
    arm_shoulder('R', arm_ready_R); 

    arm_updown(1);
    delay(100);
    for(int i = 40; i < 57; i++)
      {
         arm_hand('R', i);
         delay(10);
      }
   
    delay(100);
    
    do{robot.motor('D', -15); delay(5);} while(robot.adcRead(6) < 1500);
    robot.motor('D', 5);
    delay(10);
    robot.motor('D', 0);
    delay(200); 
    arm_hand('R', 40);
    delay(300);

  }
void go_to_mission_on_stand(char LR,  int up, int deg)
  {
    arm_updown(up+3);
    if(LR == 'L')
      {
         arm_shoulder('R', 175);  
         arm_shoulder('L', deg); 
         arm_hand('R', 40); 
      }
    if(LR == 'R')
      {
         arm_shoulder('L', 175);  
         arm_shoulder('R', deg); 
         arm_hand('L', 40); 
      }
      
    else
      {
        arm_hand('L', 40); 
        arm_hand('R', 40); 
      }
   delay(200);
   //move_stuck(20, 0.25, 1500);
   move_untrasonic_to_stand(20, 20, 0.55);
   robot.motor('D', -30);
   delay(150);
   robot.motor('D', 30);
   delay(10);
   robot.motor('D', 0);
   delay(10);
   arm_updown(up - 3);
     if(LR == 'L')
      {
          for(int i = 25; i < 60; i++)
              {
                arm_hand('L', i);                
                delay(15);
              } 
            delay(100);
            arm_hand('L', 75);
            
      }
    else if(LR == 'R')
      {
           for(int i = 25; i < 70; i++)
              {
                arm_hand('R', i);                
                delay(10);
              } 
            delay(100);
            arm_hand('R', 75);
      }
    else
      {
         for(int i = 40; i < 70; i++)
              {
                arm_hand('L', i);
                arm_hand('R', i);  
              
                delay(10);
              }
            delay(100);
      }
      delay(200);
      move_out_stand(-20, -20, 0.3, 10);
  }
  
void go_to_mission_to_can(int up, int direction)       
    {   
      // if(_target_cm > 0)
      //   {
      //     arm_updown(up); 
      //   }
        
        // move(30,  30, 1.5, 15, 0 , 0);
        if(direction < 8)
          {
            move_untrasonic(20, 20, 0.25, direction);
          }
        else
          {
            move_to_can(20,  20, 0.5, direction, 1);
          }
        
         delay(200);
         
           for(int i = 90; i > 30; i--)
              {
                arm_hand('L', i);  
                arm_hand('R', i);  
                delay(4);
              } 

        delay(300);
        arm_updown(up+2); 
        delay(200);
    }

  
void   go_mission_on_stand()
 {
  arm_ready();      
  arm_updown(6);
  go_to_mission_to_can(7, 3);  
  delay(200);
  move(-40,  -40, 1.5, 20, 0, 7);
  // arm_start_new();   
}