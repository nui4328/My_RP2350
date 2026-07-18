
void set_arm_to_can()
  {
      arm_shoulder('L', 170);  //----ปรับค่า
      arm_shoulder('R', 170);  //----ปรับค่า
      arm_can('L', 40);  // ฝ่ามือซ้ายหุบเข้า
      arm_can('R', 40);
  }
void set_hand_zero()
  {
      arm_hand('L', 35);
      arm_hand('R', 35);
  }
  
void armLR_to_can()
  {   
    delay(200); 
     arm_updown(13);     //----ปรับค่า
    
    arm_can('L', 40);
    arm_can('R', 30);

    arm_shoulder('L', 170);  //----ปรับค่า  
    arm_shoulder('R', 170);  //----ปรับค่า  

    delay(200);
    arm_updown(8);
    delay(300);
   
     for(int i = 40; i <= 60; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_hand('L', i);
        arm_hand('R', i);
        delay(5);
      }
    delay(200);
    
    arm_hand('L', 100);
    arm_hand('R', 100);
    delay(200);
    arm_updown(14);
    delay(200);
   
    for(int i = 40; i <= 100; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L', i);
        arm_can('R', i);
        delay(5);
      }
    arm_hand('L', 100);
    arm_hand('R', 100);
  
    delay(100);

    arm_can('L',100);
    arm_can('R',100);
    delay(400);
    arm_shoulder('L', arm_shoulderL_begin);  // ค่าเดิมคือ65
    arm_shoulder('R', arm_shoulderR_begin);
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
    
    arm_updown(8);
    delay(200);
    arm_hand('L', 60);
    delay(200);
    
    arm_hand('L', 100);
    delay(300);
    arm_updown(13);
   
    for(int i = 40; i <= 110; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L', i);
        delay(5);
      }
    arm_hand('L', 110);
  
    delay(100);

    arm_can('L',110);
    delay(400);
    arm_shoulder('L', arm_shoulderL_begin);  // ค่าเดิมคือ65
    arm_hand('L', 50);
    delay(100);
  }

void can_to_armL()
  {
    //arm_updown(0);
    arm_updown(8);
    delay(200);
    arm_hand('L', 100);
    arm_shoulder('L', 170);
    delay(300);
    for(int i = 90; i >= 40; i--)  //---->> 
      {
        arm_can('L',i);
        delay(5);
      }
    delay(200);
    arm_hand('L', 40);
    delay(500);
    
    arm_updown(13);
    delay(300);
    arm_can('L',90);
    arm_shoulder('L', 70);
    arm_hand('L', 40);
    delay(100);
  }


void can_to_armL_5cm()
  {
    //arm_updown(0);
    arm_updown(8);
    delay(200);
    arm_hand('L', 100);
    arm_shoulder('L', 170);
    delay(300);
    for(int i = 90; i >= 40; i--)  //---->> 
      {
        arm_can('L',i);
        delay(5);
      }
    delay(200);
    arm_hand('L', 40);
    delay(500);
    
    arm_updown(13);
    delay(300);
    arm_can('L',100);
    arm_shoulder('L', 70);
    arm_hand('L', 40);
    arm_updown(9);
    delay(100);
  }

void can_to_armL_10cm()
  {
    //arm_updown(0);
    arm_updown(8);
    delay(200);
    arm_hand('L', 100);
    arm_shoulder('L', 170);
    delay(300);
    for(int i = 90; i >= 40; i--)  //---->> 
      {
        arm_can('L',i);
        delay(5);
      }
    delay(200);
    arm_hand('L', 40);
    delay(500);
    
    arm_updown(15);
    delay(300);
    arm_can('L',90);
    arm_shoulder('L', 70);
    arm_hand('L', 40);
    delay(100);
  }

 void can_to_armL_15cm()
  {
    //arm_updown(0);
    arm_updown(8);
    delay(200);
    arm_hand('L', 100);
    arm_shoulder('L', 170);
    delay(300);
    for(int i = 90; i >= 40; i--)  //---->> 
      {
        arm_can('L',i);
        delay(5);
      }
    delay(200);
    arm_hand('L', 40);
    delay(500);
    
    arm_updown(18);
    delay(300);
    arm_can('L',90);
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
    
    arm_updown(8);
    delay(200);
    arm_hand('R', 60);
    delay(200);
    
    arm_hand('R', 110);
    delay(300);
    arm_updown(13);
   
    for(int i = 40; i <= 110; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('R', i);
        delay(5);
      }
    arm_hand('R', 110);
  
    delay(100);

    arm_can('R',110);
    delay(400);
    arm_shoulder('R', 70);
    arm_hand('R', 50);
    delay(100);
  }

void can_to_armR()
  {    
    arm_updown(8);
    delay(200);
    arm_hand('R', 110);
    arm_shoulder('R', 170);
    delay(300);
    for(int i = 90; i >= 40; i--)  //---->> 
      {
        arm_can('R',i);
        delay(5);
      }
    delay(200);
    arm_hand('R', 40);
    delay(300);
    arm_updown(14);
    delay(300);
    arm_can('R',100);
    arm_shoulder('R', 70);
    arm_hand('R', 40);
    delay(100);
 
  }

void can_to_armR_5cm()
  {    
    arm_updown(8);
    delay(200);
    arm_hand('R', 110);
    arm_shoulder('R', 170);
    delay(300);
    for(int i = 90; i >= 40; i--)  //---->> 
      {
        arm_can('R',i);
        delay(5);
      }
    delay(200);
    arm_hand('R', 40);
    delay(300);
    arm_updown(14);
    delay(300);
    arm_can('R',100);
    arm_shoulder('R', 70);
    arm_hand('R', 40);
    arm_updown(9);
    delay(100);
 
  }

void can_to_armR_10cm()
  {    
    arm_updown(7);
    delay(200);
    arm_hand('R', 110);
    arm_shoulder('R', 170);
    delay(300);
    for(int i = 90; i >= 40; i--)  //---->> 
      {
        arm_can('R',i);
        delay(5);
      }
    delay(200);
    arm_hand('R', 40);
    delay(300);
    arm_updown(14);
    delay(300);
    arm_can('R',100);
    arm_shoulder('R', 70);
    arm_hand('R', 40);
    delay(100);
 
  }

  void can_to_armR_15cm()
  {    
    arm_updown(7);
    delay(200);
    arm_hand('R', 110);
    arm_shoulder('R', 170);
    delay(300);
    for(int i = 90; i >= 40; i--)  //---->> 
      {
        arm_can('R',i);
        delay(5);
      }
    delay(200);
    arm_hand('R', 40);
    delay(300);
    arm_updown(17);
    delay(300);
    arm_can('R',90);
    arm_shoulder('R', 70);
    arm_hand('R', 40);
    delay(100);
 
  }