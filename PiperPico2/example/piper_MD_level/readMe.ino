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
  
   move_fw(30, 1.2, 60, "none_line");
    turnGyro(60, -90);
    set_b(2);
    move_fw(30, 1.2, 55, "none_line");
    turnGyro(60, 90);
    move_fw(30, 1.2, 55, "line");
    turnGyro(60, 90);
    set_b(2);
    move_fw(30, 1.2, 80, "line");
    delay(200);
    set_f(2);
    move_bw(30, 1.2, 80, "line");
    turnGyro(60, 90);
    set_b(2);
    
    move_fw(30, 1.2, 60, "none_line");
    turnGyro(60, -90);
    set_b(2);

    move_fw(30, 1.2, 60, "line");
    turnGyro(60, 90);
    move_fw(30, 1.2, 60, "line");

  

*/