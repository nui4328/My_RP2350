
void armLR_to_can()
  {    
    arm_updown(2);
    arm_hand('L', 60);
    arm_hand('R', 60);
    delay(200);
    arm_updown(0);
    arm_hand('L', 30);
    arm_hand('R', 30);
    delay(200);
    arm_updown(10);
    //sw();          //-->> รอกุดปุ่ม
    arm_can('L', 78);
    arm_can('R', 78);
    arm_shoulder('L', 162);  //----ปรับค่า 
    arm_shoulder('R', 159);  //----ปรับค่า 
    delay(300);
    //sw();
    arm_updown(8);
    //sw();          //-->> รอกุดปุ่ม

    delay(200);
    arm_hand('L', 55);
    arm_hand('R', 55);
    delay(300);
     arm_shoulder('L', 159);  //----ปรับค่า 
    arm_shoulder('R', 154);  //----ปรับค่า
    arm_hand('L', 70);
    arm_hand('R', 70);
    delay(200);
    arm_updown(14);

    for(int i = 78; i <= 85; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L', i);
        arm_can('R', i);
        delay(7);
      }
    arm_hand('L', 120);
    arm_hand('R', 120);
    delay(200);
     for(int i = 85; i <= 130; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L', i);
        arm_can('R', i);
        delay(7);
      }
    for(int i = 130; i <= 160; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L', i);
        arm_can('R', i);
        delay(3);
      }
    delay(400);
    arm_shoulder('L', 70);
    arm_shoulder('R', 70);
    arm_hand('L', 50);
    arm_hand('R', 50);
  }
void armL_to_can()
  {
    arm_updown(2);    
    arm_hand('L', 50);
    delay(200);
    arm_updown(0);
    arm_hand('L', 20);
    delay(200);

    arm_updown(13);     //----ปรับค่า
    
    arm_can('L', 75);
    arm_shoulder('L', 162);  //----ปรับค่า  
    delay(300);

    arm_updown(7);
    delay(200);
    arm_hand('L', 40);
    delay(200);
    
    arm_hand('L', 130);
    delay(300);
    arm_updown(17);
    for(int i = 78; i <= 85; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L', i);
        delay(7);
      }
    arm_hand('L', 120);
    delay(200);
     for(int i = 85; i <= 130; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L', i);
        delay(7);
      }
    for(int i = 130; i <= 160; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('L', i);
        delay(3);
      }
    delay(400);

    arm_can('L',160);
    delay(400);
    arm_shoulder('L', 70);
    arm_hand('L', 50);
  }
void can_to_armL()
  {
    arm_updown(0);
    arm_updown(7);
    arm_hand('L', 120);
    arm_shoulder('L', 162);
    delay(300);

    for(int i = 160; i >= 78; i--)  //---->> 
      {
        robot.setservo(4, i);
        delay(10);
      }

    delay(400);
    arm_hand('L', 30);
    delay(200);
    arm_updown(11);
    delay(200);
    arm_can('L',150);
    arm_shoulder('L', 65);
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
    //sw();          //-->> รอกุดปุ่ม
    arm_can('R', 75);
    arm_shoulder('R', 159);  //----ปรับค่า 
    delay(300);

    arm_updown(6);
    //sw();          //-->> รอกุดปุ่ม

    delay(200);
    arm_hand('R', 40);
    delay(200);
    arm_hand('R', 120);
    delay(300);
    arm_updown(17);

    for(int i = 78; i <= 85; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('R', i);
        delay(7);
      }
    arm_hand('R', 120);
    delay(200);
     for(int i = 85; i <= 130; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('R', i);
        delay(7);
      }
    for(int i = 130; i <= 160; i++)  //---->> กลับเข้าเก็บช้า
      {
        arm_can('R', i);
        delay(3);
      }
    delay(400);
    arm_shoulder('R', 70);
    arm_hand('R', 50);
  }
void can_to_armR()
  { 
    arm_updown(0);   
    arm_updown(5);
    arm_hand('R', 120);
    arm_shoulder('R', 159);
    delay(300);

    for(int i = 150; i >= 75; i--)  //---->> 
      {
        arm_can('R', i);
        delay(10);
      }
    arm_hand('R', 40);
    delay(300);
    arm_updown(9);
    delay(200);
    arm_can('R',150);
    arm_shoulder('R', 70);
    arm_hand('R', 35);
  }