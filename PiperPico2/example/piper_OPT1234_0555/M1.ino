


void s1()
  {
    arm_updown(0);
    arm_updown(5);
    arm_shoulder('L', 65);
    arm_shoulder('R', 65);
    arm_hand('L', 80);
    arm_hand('R', 80);
    move_adc(20, 1.8, 750);
    delay(100);
    arm_hand('L', 40);
    arm_hand('R', 40);
    delay(100);
    arm_updown(9);
    delay(200);
    move(-40,  -40, 2.4, 10, 9);
    move(-60,  -60, 2.4, 55, 5);
    turnGyro(80, -90); 

    setrobot_bw(3);   //----->>ถอยหลังเซต

    turnGyro(80, -90); 

    armLR_to_can();


    arm_updown(0);
    arm_shoulder('L', 65);
    arm_shoulder('R', 65);
    arm_hand('L', 80);
    arm_hand('R', 80);
    move_untrasonic(20, 20, 1.8, 5);   //----->>เดินเข้าไปคีบ
    delay(100);
    arm_hand('L', 40);
    arm_hand('R', 40);
    delay(100);

    move(-30,  -30, 2.4, 25, 2);
    turnGyro(80, 90);
    setrobot_bw(2);

    move(60,  60, 2.4, 30, 2);
    turnGyro(80, -90);

    
    arm_updown(12);
    arm_shoulder('R', 170);
     
    move_stuck(20, 1.5, 500);   //----->>เดินเข้าไปวางบนแท่น
    arm_shoulder('L', 90);
    arm_updown(11);
    arm_hand('L', 90);
    delay(100);

    move(-40,  -40, 2.4, 15, 11);
    can_to_armL();

    arm_updown(0);       //--------------->> ลง 0 แล้วคีบใหม่
    arm_hand('L', 40);
    delay(300);
    arm_hand('L', 40);
    delay(100);

    arm_updown(14); 
    arm_shoulder('L', 20);
    move_stuck(20, 1.5, 500);  //----->>เดินเข้าไปวางบนแท่น
    
    arm_updown(11);
    arm_hand('L', 90);
    delay(100);

    move(-30,  -30, 2.4, 25, 11);


    //
    armR_to_armL();
    arm_updown(0);
    arm_updown(1);
    move(-50,  -50, 2.4, 45, 2);
    turnGyro(80, -90);
    setrobot_bw(2);   //----->>ถอยหลังเซต
    turnGyro(80, -90);

    can_to_armR();
    arm_updown(0);       //--------------->> ลง 0 แล้วคีบใหม่
    arm_hand('R', 40);
    delay(300);
    arm_hand('R', 40);
    delay(100);

    arm_updown(9);
    
    move_stuck(20, 1.5, 500);   //----->>เดินเข้าไปวางบนแท่น
    arm_updown(7);
    arm_hand('L', 80);
    arm_hand('R', 80);
    move(-30,  -30, 2.4, 15, 5);










  }