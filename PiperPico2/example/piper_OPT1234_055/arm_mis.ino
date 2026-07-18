

void arm_ready()       // เตียมมือจับตอนเข้าคีบ
  {
    arm_updown(0);
    arm_shoulder('L', arm_shoulderL_begin - 5);  // ค่าเดิมคือ65
    arm_shoulder('R', arm_shoulderR_begin - 5);

    arm_hand('L', 85);
    arm_hand('R', 75);
  }

void arm_start_new()
  {
    arm_shoulder('L', arm_shoulderL_begin);  // ค่าเดิมคือ65
    arm_shoulder('R', arm_shoulderR_begin); 

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
    delay(300);

  }

  void arm_start_new_L()
  {
     arm_shoulder('L', arm_shoulderL_begin);  // ค่าเดิมคือ65
    arm_shoulder('R', arm_shoulderR_begin);
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
     arm_shoulder('L', arm_shoulderL_begin);  // ค่าเดิมคือ65
    arm_shoulder('R', arm_shoulderR_begin);

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
void go_to_mission_on_stand(char LR, int deg)
  {
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
   delay(100);
   //move_stuck(20, 0.25, 1500);
   move_untrasonic_to_stand(25, 25, 1.5, 10);   // ความเร็วซ้าย  ความเร็วขวา  ค่า kp  ระยะของอัลต้าโซนิค
   if(deg != arm_shoulderL_begin)
    {
      move_untrasonic_to_stand(20, 20, 1.5, 3);
    }
  else  {
    move_untrasonic_to_stand(20, 20, 1.5, 4);
  }
  
  arm_updown(last_target_cm - 3);   
  delay(200);
     if(LR == 'L')
      {
          for(int i = 30; i < 55; i++)
              {
                arm_hand('L', i);                
                delay(10);
              } 
            delay(100);
            arm_hand('L', 70); 
            
      }
    else if(LR == 'R')
      {
           for(int i = 30; i < 55; i++)
              {
                arm_hand('R', i);                
                delay(10);
              } 
            arm_hand('R', 70); 
            delay(100);
      }
    else
      {
         for(int i = 30; i < 70; i++)
              {
                arm_hand('L', i);
                arm_hand('R', i);  
              
                delay(10);
              }
            delay(100);
      }
      delay(200);
      move_out_stand(-20, -20, 0.3, 25);
  }
  
void go_mission_on_floor()     
    {   
      arm_updown(2);
      //move_untrasonic(20, 20, 2.25, 12);
       move_untrasonic(18, 18, 2.25, 7);
         delay(200);
         arm_updown(0);
         delay(300);
         
           for(int i = 70; i > 35; i--)
              {
                arm_hand('L', i);  
               arm_hand('R', i);  
                delay(5);
              } 

         delay(300);
         arm_updown(4); 
         delay(200);
    
    }

  
void   go_mission_on_stand()
 {

  arm_ready();      
  arm_updown(5);
  move_untrasonic(20, 20, 2.25, 4);
  delay(300);
  for(int i = 70; i > 35; i--)
              {
                arm_hand('L', i);  
               arm_hand('R', i);  
                delay(5);
              } 
  delay(300);
  arm_updown(10);
  delay(300);
  move(-40,  -40, 1.5, 20, 0, 10);

}