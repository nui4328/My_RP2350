
void arm_ready()       // เตียมมือจับตอนเข้าคีบ
  {
    arm_updown(0);
    arm_shoulder('L', 80);  // ค่าเดิมคือ65
    arm_shoulder('R', 60);

    arm_hand('L', 70);
    arm_hand('R', 70);
  }

void arm_start_new()
  {
    arm_shoulder('L', 83);  // ค่าเดิมคือ65
    arm_shoulder('R', 63); 

    arm_updown(4);
    delay(100);
    for(int i = 40; i < 57; i++)
      {
         arm_hand('L', i);  
         delay(10);
      }
    arm_updown(0);
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
    arm_shoulder('L', 83);  // ค่าเดิมคือ65
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
    
    arm_updown(0);
    delay(300);
    arm_hand('L', 40);  
    delay(300);

  }
   void arm_start_new_R()
  {
    arm_shoulder('L', 83);  // ค่าเดิมคือ65
    arm_shoulder('R', 63); 

    arm_updown(1);
    delay(100);
    for(int i = 40; i < 57; i++)
      {
         arm_hand('R', i);
         delay(10);
      }
   
    delay(100);
    
    arm_updown(0);
    delay(200); 
    arm_hand('R', 40);
    delay(300);

  }
void go_to_mission_on_stand(char LR,  int up, int deg)
  {
    arm_updown(up);
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
   move_stuck(20, 0.25, 1500);
   move(-20,  -20, 1.5, 5, 1 , up);
   arm_updown(up - 2);
     if(LR == 'L')
      {
          for(int i = 40; i < 70; i++)
              {
                arm_hand('L', i);                
                delay(10);
              } 
            delay(100);
            
      }
    else if(LR == 'R')
      {
           for(int i = 40; i < 70; i++)
              {
                arm_hand('R', i);                
                delay(10);
              } 
            delay(100);
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
  }
  
void go_to_mission_to_can(int up, int direction)       
    {   
        arm_updown(up); 
        // move(30,  30, 1.5, 15, 0 , 0);
        if(direction < 8)
          {
            move_untrasonic(20, 20, 0.25, direction);
          }
        else
          {
            move(20,  20, 0.5, direction, 10 , 0);
          }
        
         delay(200);
         
           for(int i = 90; i > 45; i--)
              {
                arm_hand('L', i);  
               arm_hand('R', i);  
                delay(4);
              } 

            delay(100);
        arm_updown(up+2);  
    }