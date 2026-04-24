
void armLR_to_can()
  {
    arm_updown(3);    
    arm_hand('L', 40);
    arm_hand('R', 40);
    delay(200);
    arm_updown(0);
    arm_hand('L', 20);
    arm_hand('R', 20);
    delay(200);
    arm_updown(11);
    
    arm_can('L', 70);
    arm_can('R', 70);
    arm_shoulder('L', 180);
    arm_shoulder('R', 180);
    delay(50);
    arm_updown(7);
    delay(200);
    arm_hand('L', 40);
    arm_hand('R', 40);
    delay(200);
    arm_hand('L', 120);
    arm_hand('R', 120);
    delay(300);

    for(int i = 70; i <= 160; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L',i);
        arm_can('R',i);
        delay(2);
      }

    
    delay(400);
    arm_shoulder('L', 70);
    arm_shoulder('R', 70);
    arm_hand('L', 30);
  }
void armL_to_can()
  {
    arm_updown(2);    
    arm_hand('L', 50);
    delay(200);
    arm_updown(0);
    arm_hand('L', 20);
    delay(200);

    sw();

    arm_updown(11);     //----ปรับค่า
    
    arm_can('L', 75);
    arm_shoulder('L', 150);  //----ปรับค่า  
    delay(300);
    sw();

    arm_updown(7);
    delay(200);
    arm_hand('L', 40);
    delay(200);
    arm_hand('L', 120);
    delay(300);

    for(int i = 70; i <= 160; i++)  //---->> กลับเข้าเก็บช้า
      {
        robot.setservo(4, i);
        delay(3);
      }

    arm_can('L',160);
    delay(400);
    arm_shoulder('L', 70);
    arm_hand('L', 30);
  }
void can_to_armL()
  {
    arm_updown(0);
    arm_updown(7);
    arm_hand('L', 120);
    arm_shoulder('L', 180);
    delay(300);

    for(int i = 160; i >= 70; i--)  //---->> 
      {
        robot.setservo(4, i);
        delay(3);
      }

    delay(400);
    arm_hand('L', 10);
    delay(200);
    arm_updown(11);
    delay(200);
    arm_can('L',160);
    arm_shoulder('L', 70);
    arm_hand('L', 25);
  }

  
void armR_to_can()
  {
    arm_updown(2);
    arm_hand('R', 50);
    delay(200);
    arm_updown(0);
    arm_hand('R', 20);
    delay(200);
    arm_updown(11);
    sw();          //-->> รอกุดปุ่ม
    arm_can('R', 75);
    arm_shoulder('R', 150);  //----ปรับค่า 
    delay(300);

    arm_updown(7);
    sw();          //-->> รอกุดปุ่ม

    delay(200);
    arm_hand('R', 40);
    delay(200);
    arm_hand('R', 120);
    delay(300);

    for(int i = 70; i <= 160; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('R', i);
        delay(3);
      }

    delay(400);
    arm_shoulder('R', 70);
    arm_hand('R', 30);
  }
void can_to_armR()
  { 
    arm_updown(0);   
    arm_updown(4.5);
    arm_hand('R', 120);
    arm_shoulder('R', 180);
    delay(300);

    for(int i = 160; i >= 70; i--)  //---->> 
      {
        arm_can('R', i);
        delay(3);
      }

    delay(400);
    arm_hand('R', 10);
    delay(200);
    arm_updown(11);
    delay(200);
    arm_can('R',160);
    arm_shoulder('R', 70);
    arm_hand('R', 25);
  }