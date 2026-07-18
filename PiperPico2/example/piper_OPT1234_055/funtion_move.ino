void s1()
  {
    setrobot_bw_gyroset_zero(2);   //---เริ่มต้นหุ่นยนต์ใหม่ เซต gyro เป็นทิศ 0
    setrobot_bw(2);

    move(40,  40, 1.5, 30, 0, 2);
    setrobot_fw(2);   //----->>ถอยหลังเซต
    turnGyro(70, 90, true);   //----->>หมุนตามทิศ
    setrobot_bw(2);   //----->>ถอยหลังเซต

    move(40,  40, 1.5, 30, 10, 2);
    turnGyro(70, 90, true); 
    setrobot_bw(2);   //----->>ถอยหลังเซต

    move(70, 70, 1.5, 60, 0, 2);
    setrobot_fw(2);   //----->>ถอยหลังเซต
    turnGyro(70, 90, true); 
    setrobot_bw(2);   //----->>ถอยหลังเซต

    move(70,  70, 1.5, 60, 10, 2);
    turnGyro(70, 90, true); 
    setrobot_bw(2);   //----->>ถอยหลังเซต

     move(-40,  -40, 1.5, 30, 0, 2);
    setrobot_fw(2);   //----->>ถอยหลังเซต
    turnGyro(70, 90, true); 
    setrobot_bw(2);   //----->>ถอยหลังเซต

    move(-40,  -40, 1.5, 30, 10, 2);
    turnGyro(70, 90, true); 
    setrobot_bw(2);   //----->>ถอยหลังเซต

    move(-70, -70, 1.5, 60, 0, 2);
    setrobot_fw(2);   //----->>ถอยหลังเซต
    turnGyro(70, 90, true); 
    setrobot_bw(2);   //----->>ถอยหลังเซต

    move(-70,  -70, 1.5, 60, 10, 2);
    turnGyro(70, 90, true); 
    setrobot_bw(2);   //----->>ถอยหลังเซต




    move(40,  40, 1.5, 30, 5, 1);
    turnGyro(70, 180, true); 
    setrobot_bw(2);   //----->>ถอยหลังเซต
    arm_bridge();
    move(40,  40, 1.5, 50, 5, 4);

    setrobot_fw(2);   //----->>ถอยหลังเซต
    turnGyro(70, -90, true); 
    move(-40,  -40, 1.5, 50, 5, 4);
    setrobot_bw(2);   //----->>ถอยหลังเซต

    

    move_chopsticks(35,  35, 1.5, 70, 5, 2); // ข้ามตะเกียบ
    






  }