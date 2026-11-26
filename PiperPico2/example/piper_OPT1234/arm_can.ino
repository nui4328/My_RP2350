
void armLR_to_can()
  {   
    delay(200); 
     arm_updown(15);     //----ปรับค่า
    
    arm_can('L', 40);
    arm_can('R', 40);

    arm_shoulder('L', 170);  //----ปรับค่า  
    arm_shoulder('R', 170);  //----ปรับค่า  
    delay(300);
    
    arm_updown(10);
    delay(200);
    arm_hand('L', 60);
    arm_hand('R', 60);
    delay(200);
    
    arm_hand('L', 110);
    arm_hand('R', 110);
    delay(300);
    arm_updown(17);
   
    for(int i = 40; i <= 120; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L', i);
        arm_can('R', i);
        delay(5);
      }
    arm_hand('L', 110);
    arm_hand('R', 110);
  
    delay(100);

    arm_can('L',118);
    arm_can('R',118);
    delay(400);
    arm_shoulder('L', 70);
    arm_shoulder('R', 70);
    arm_hand('L', 50);
    arm_hand('R', 50);
    delay(100);

  }


void armL_to_can()
  {
    arm_updown(13);     //----ปรับค่า
    
    arm_can('L', 40);

    arm_shoulder('L', 170);  //----ปรับค่า  
    delay(300);
    
    arm_updown(11);
    delay(200);
    arm_hand('L', 60);
    delay(200);
    
    arm_hand('L', 110);
    delay(300);
    arm_updown(15);
   
    for(int i = 40; i <= 120; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L', i);
        delay(5);
      }
    arm_hand('L', 110);
  
    delay(100);

    arm_can('L',118);
    delay(400);
    arm_shoulder('L', 70);
    arm_hand('L', 50);
    delay(100);
  }
void can_to_armL()
  {
    arm_updown(0);
    arm_updown(10);
    delay(200);
    arm_hand('L', 110);
    arm_shoulder('L', 170);
    delay(300);
    for(int i = 110; i >= 40; i--)  //---->> 
      {
        arm_can('L',i);
        delay(5);
      }
    delay(200);
    arm_hand('L', 40);
    delay(300);
    arm_updown(18);
    delay(300);
    arm_can('L',110);
    arm_shoulder('L', 70);
    arm_hand('L', 40);
    delay(100);
  }

  
void armR_to_can()
  {
    arm_updown(13);     //----ปรับค่า
    
    arm_can('R', 40);

    arm_shoulder('R', 170);  //----ปรับค่า  
    delay(300);
    
    arm_updown(10);
    delay(200);
    arm_hand('R', 60);
    delay(200);
    
    arm_hand('R', 110);
    delay(300);
    arm_updown(15);
   
    for(int i = 40; i <= 120; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('R', i);
        delay(5);
      }
    arm_hand('R', 110);
  
    delay(100);

    arm_can('R',118);
    delay(400);
    arm_shoulder('R', 70);
    arm_hand('R', 50);
    delay(100);
  }
void can_to_armR()
  { 
    arm_updown(0);
    arm_updown(8);
    delay(200);
    arm_hand('R', 110);
    arm_shoulder('R', 170);
    delay(300);
    for(int i = 110; i >= 35; i--)  //---->> 
      {
        arm_can('R',i);
        delay(5);
      }
    delay(200);
    arm_hand('R', 40);
    delay(300);
    arm_updown(14);
    delay(300);
    arm_can('R',110);
    arm_shoulder('R', 70);
    arm_hand('R', 40);
    delay(100);
 
  }