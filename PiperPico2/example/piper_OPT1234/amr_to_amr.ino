//arm_hand('L', 30);  // ฝ่ามือซ้ายหุบเข้า
//arm_hand('R', 30);  // ฝ่ามือซ้ายหุบเข้า


void armL_to_armR()      //------->> เปลี่ยนกระป๋องจากมือซ้ายไปมือขวา
  {
    arm_shoulder('L', 90);
    arm_hand('L', 30);  // ฝ่ามือซ้ายหุบเข้า
    arm_hand('R', 60);
    arm_shoulder('R', 20);
    delay(500);
    
    for(int i = 70; i >= 40; i--)  //---->> 
      {
        arm_shoulder('L',i);
        delay(3);
      }
    delay(300);
    arm_hand('R',40);
    
 
    delay(300);
    arm_hand('L', 90);
    delay(300);
    
    arm_shoulder('L', 20);
    arm_shoulder('R', 90);
    delay(200);
    arm_shoulder('L', 83);
    arm_hand('L', 50);
    delay(200);
  }

void armR_to_armL()  //------->> เปลี่ยนกระป๋องจากมือขวาไปมือซ้าย
  {
    arm_shoulder('R', 90);
    arm_hand('R', 40);  // ฝ่ามือซ้ายหุบเข้า
    arm_hand('L', 60);
    arm_shoulder('L', 45);
    delay(300);
    
    for(int i = 70; i >= 15; i--)  //---->>     //-----   
      {
        arm_shoulder('R',i);
        delay(3);
      }
    delay(300);

    arm_hand('L',40);   
    delay(300);
    arm_hand('R', 80);
    delay(200);
    
    arm_shoulder('L', 90);
    delay(200);
    arm_shoulder('R', 83);
    arm_hand('R', 50);
    delay(200);
  }
