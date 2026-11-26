//arm_hand('L', 30);  // ฝ่ามือซ้ายหุบเข้า
//arm_hand('R', 30);  // ฝ่ามือซ้ายหุบเข้า

void armL_to_armR()      //------->> เปลี่ยนกระป๋องจากมือซ้ายไปมือขวา
  {
    arm_shoulder('L', 80);
    arm_hand('L', 20);  // ฝ่ามือซ้ายหุบเข้า
    arm_hand('R', 70);
    arm_shoulder('R', 5);
    delay(200);

    arm_shoulder('L', 5);
    delay(300);
    arm_hand('R',30);
    
    delay(300);
    arm_hand('L', 90);
    delay(100);
    
    arm_shoulder('R', 90);
    delay(200);
    arm_shoulder('L', 70);
    arm_hand('L', 50);
    delay(200);
  }

void armR_to_armL()  //------->> เปลี่ยนกระป๋องจากมือขวาไปมือซ้าย
  {
    arm_shoulder('R', 80);
    arm_hand('R', 20);  // ฝ่ามือซ้ายหุบเข้า
    arm_hand('L', 80);
    arm_shoulder('L', 10);
    delay(200);

    arm_shoulder('R', 5);
    delay(300);
    arm_hand('L',30);
    
    delay(300);
    arm_hand('R', 90);
    delay(100);
    
    arm_shoulder('L', 90);
    delay(200);
    arm_shoulder('R', 70);
    arm_hand('R', 50);
    delay(200);
  }
