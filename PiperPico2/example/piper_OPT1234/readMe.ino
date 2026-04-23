/*
  robot.motor('A', 60);   robot.motor('C', 60);
  robot.motor('B', 60);   robot.motor('D', 60);

  robot.setServo(0, 180);
  robot.setServo(1, 180);
  robot.setServo(2, 180);
  robot.setServo(3, 180);
  robot.setServo(4, 180);

  robot.showVoltageUntilButton();  //จอแสดง V
  robot.setMotorPWMFrequency(20000);  // // ถ้าต้องการเปลี่ยนความถี่ PWM มอเตอร์ (เช่น สำหรับ coreless)  //20 kHz// หรือ 25000 ถ้าต้องการเงียบที่สุด

  Serial.print(encoder1.Poss_L());Serial.print("  "); Serial.print(encoder1.Poss_R());
  Serial.print("  ");
  Serial.print(encoder2.Poss_L());Serial.print("  "); Serial.println(encoder2.Poss_R());

  robot.knopRead(); //--->> อ่านค่า ปุ่มหมุน (knop)
  playTone(3000, 300);

  Serial.print(robot.adcRead(0));   // 0-9
  Serial.print(robot.adcMax(0));   // 0-9
  Serial.print(robot.adcMin(0));   // 0-9


  turnGyro(90, -90);   // ทดสอบหมุนซ้าย 90 องศา

  arm_updown(0);
  arm_updown(10);
  
  arm_updown(0);
   
    move(60,  60, 2.4, 60, 0);
    turnGyro(80, 90); 
    move(60,  60, 2.4, 60, 1);
    turnGyro(80, 90); 
    move(60,  60, 2.4, 60, 1);
    turnGyro(80, 90); 
    move(80,  80, 2.4, 60, 1);
    turnGyro(80, 90); 

    move(30,  30, 2.4, 30, 5);
    delay(1500);
    move(-30,  -30, 2.4, 15, 5);
    move(-30,  -30, 2.4, 25, 0);
    turnGyro(80, 90); 
    

    can_to_armR();
    armR_to_armL();
    armL_to_can(); 
    can_to_armL();
    armL_to_armR();
    armR_to_can();

    for(int i=0; i<5; i++)
    {
    arm_updown(0);
    arm_hand('L', 30);
    arm_hand('R', 30);
    delay(500);
    armL_to_can();
    delay(500);
    can_to_armL();

    arm_updown(1);
    arm_hand('L', 40);
    arm_hand('R', 40);


     move_stuck(25, 1.5, 500);  //----เดินชนแท่น


    }
    



*/