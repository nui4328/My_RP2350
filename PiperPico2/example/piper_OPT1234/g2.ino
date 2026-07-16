void mission_2() 
  {
    move(40,  40, 1.5, 30, 15, 1);               //---------เดินไปถึงบล๊อก เพื่อเดินต่อเข้าเส้น  
    
    arm_ready();
    move(30,  30, 1.5, 15, 0, 1);
    go_to_mission_to_can(0, 6);  // 0 คือ ระยะยกขึ้น  0 กระป๋องวางที่พื้น     , ระยะ 7
    armLR_to_can();
    move(-30,  -30, 1.5, 10, 15, 5);
    arm_bridge();

    setrobot_fw(1);
    arm_updown(5);
 
    move(-30,  -30, 1.5, 30, 15, 5);
    turnGyro(75, -90); 
    setrobot_fw(2);

    move(-80, -80, 1.5, 80, 15, 5);
    setrobot_bw(1);
    arm_updown(0);

    turnGyro(75, 90); 
    setrobot_bw(1);

    arm_ready();
    go_mission_on_stand(); //เข้าคีบบนแท่น 5 เซนติเมตร
    move(-30,  -30, 1.5, 10, 15, 5);
    
    setrobot_bw(2);
    turnGyro(75, -90); 
    setrobot_bw(1);

    move(30,  30, 1.5, 28, 15, 3);
    turnGyro(75, 90); 
    setrobot_bw(2);

    arm_bridge_box();

    move_bridge(40,  40, 1.5, 60, 0, 3);

    setrobot_fw(1);
    turnGyro(75, -90); 
    

    move_bridge(40, 40, 1.5, 60, 0, 3);

    turnGyro(75, -90); 
    setrobot_fw(1);

    move(-30,  -30, 1.5, 29, 0, 3);
    setrobot_bw(1);
    turnGyro(75, -90); 
    setrobot_bw(1);

    move(80, 80, 1.5, 60, 15, 3);
    setrobot_fw(1);

    turnGyro(75, -90); 
    setrobot_bw(1);
    
    move_chopsticks(40, 40, 1.5, 65, 3);
    turnGyro(75, -90); 
    move(-30,  -30, 1.5, 30, 15, 3);
    setrobot_bw(1);
    turnGyro(75, -90); 
    setrobot_bw(1);

    move(60,  60, 1.5, 55, 15, 3);
    turnGyro(75, 90); 
    setrobot_bw(1);
    turnGyro(75, -90); 
    
    arm_start_new();
    go_to_mission_on_stand('R',  14, 80);

    move(-60,  -60, 1.5, 15, 0, 14);
    armL_to_armR();
    can_to_armL();
    
    arm_start_new();
    go_to_mission_on_stand('L',  14, 80);    // L คือเอามือซ้ายเข้าไปวาง   5 คือ ระยะที่ยกแขนขึ้น 80 องศาวาง
    move(-30,  -30, 1.5, 15, 0, 14);
    armR_to_armL();
    armL_to_can();

    move(-60, -60, 1.5, 60, 15, 3);
    setrobot_bw(1);
    turnGyro(75, 90); 
    setrobot_bw(1);

    move(30,  30, 1.5, 30, 15, 1);
    turnGyro(75, -90); 
    move(-30,  -30, 1.5, 30, 15, 1);
    setrobot_bw(1);

    turnGyro(75, 90); 
    move(30,  30, 1.5, 30, 15, 1);

    arm_ready();
    move(30,  30, 1.5, 13, 0, 1);
    go_to_mission_to_can(0, 6);  // 0 คือ ระยะยกขึ้น  0 กระป๋องวางที่พื้น     , ระยะ 7
    arm_updown(10);

    setrobot_fw(1);

    move(-60,  -60, 1.5, 58, 15, 7);
    arm_updown(0);
    turnGyro(75, -90); 
    setrobot_bw(1);

    move(30,  30, 1.5, 30, 15, 1);
    turnGyro(75, -90); 
    move(-80, -80, 1.5, 55, 15, 1);
    setrobot_bw(1);

    turnGyro(75, 90); 
    setrobot_bw(1);

    arm_start_new();
    go_to_mission_on_stand('L',  9, 85);    // L คือเอามือซ้ายเข้าไปวาง   5 คือ ระยะที่ยกแขนขึ้น 80 องศาวาง
    move(-30,  -30, 1.5, 15, 0, 9);

    can_to_armL();
    
    go_to_mission_on_stand('L',  9, 35);    // L คือเอามือซ้ายเข้าไปวาง   5 คือ ระยะที่ยกแขนขึ้น 80 องศาวาง
    move(-30,  -30, 1.5, 15, 0, 9);
    arm_ready();       // เตียมมือจับตอนเข้าคีบ

    setrobot_bw(1);


    turnGyro(75, -90); 
    setrobot_bw(1);

    move(30,  30, 1.5, 60, 15, 3);
    turnGyro(75, 90); 
    arm_bridge();

    move_chopsticks(40, 40, 1.5, 65, 3);
    turnGyro(75, -90);
    setrobot_fw(1);

    move(-30,  -30, 1.5, 60, 15, 1);
    setrobot_bw(1);
    turnGyro(75, 90);

    move(30,  30, 1.5, 25, 15, 1);
    setrobot_fw(2);
    turnGyro(75, -90);

    move_bridge(40,  40, 1.5, 60, 15, 3);
    turnGyro(75, -90);
    setrobot_fw(1);

    move_bridge(-40,  -40, 1.5, 55, 0, 3);
    setrobot_bw(1);

    turnGyro(75, -90);
    move(30,  30, 1.5, 30, 15, 1);

    turnGyro(75, 90);
    setrobot_bw(1);

    armR_to_armL();
    can_to_armR();

    arm_start_new();
    go_to_mission_on_stand('T',  19, 80);    // L คือเอามือซ้ายเข้าไปวาง   5 คือ ระยะที่ยกแขนขึ้น 80 องศาวาง
    move(-30,  -30, 1.5, 15, 0, 19);
    setrobot_bw(1);

    turnGyro(75, 90);
    move(-30,  -30, 1.5, 25, 15, 1);
    setrobot_bw(1);
    
    sw();

    turnGyro(75, 90);
    move(-30,  -30, 1.5, 30, 15, 1);
    setrobot_bw(1);

    turnGyro(75, -90);
    move(-40,  -40, 1.5, 30, 15, 1);               //---------เดินไปถึงบล๊อก เพื่อเดินต่อเข้าเส้น  

  





    






    // turnGyro(75, 90);

    // arm_start_new();
    // go_to_mission_on_stand('R', 20, 70);  // L คือเอามือซ้ายเข้าไปวาง   5 คือ ระยะที่ยกแขนขึ้น 80 องศาวาง
    // move(-40, -40, 1.5, 15, 0, 19);
    // arm_updown(10);
    // armL_to_armR();  //------->> เปลี่ยนกระป๋องจากมือ ซ้าย ไปมือ ขวา

    // can_to_armL();  //เอากระป๋องจากที่ช่องซ้าย ไปให้มือซ้าย
    // arm_start_new();
    // go_to_mission_on_stand('L', 20, 90);  // L คือเอามือซ้ายเข้าไปวาง   5 คือ ระยะที่ยกแขนขึ้น 80 องศาวาง
    // move(-40, -40, 1.5, 20, 0, 19);
    // arm_updown(3);

    // armR_to_armL();  //------->> เปลี่ยนกระป๋องจากมือ ขวา ไปมือ ซ้าย
    // armL_to_can();   //เอากระป๋องจากมือซ้าย ไปเก็บที่ช่องซ้าย
    // arm_updown(3);
    // turnGyro(75, -90);

    // setrobot_bw(2);
    // move(60, 60, 1.5, 30, 15, 3);
    // turnGyro(75, -90);

    // move(60, 60, 1.5, 30, 15, 3);

    // move_bridge(40, 40, 1.5, 65, 15, 3);
    // turnGyro(75, -90);
    // setrobot_fw(2);
    // turnGyro(75, -90);

    // move_bridge(-40, -40, 1.5, 50, 15, 3);
    // setrobot_bw(2);
    // turnGyro(75, -90);

    // move(-40, -40, 1.5, 27, 0, 2);
    // setrobot_bw(2);
    // turnGyro(75, 90);

    // setrobot_bw(2);
    // move(40, 40, 1.5, 5, 15, 3);
    // arm_ready();
    // go_to_mission_to_can(0, 7);  // 0 คือ ระยะยกขึ้น  0 กระป๋องวางที่พื้น     , ระยะ 7
    // move(-40, -40, 1.5, 15, 0, 2);

    // setrobot_bw(2);
    // turnGyro(75, -90);
    // setrobot_bw(2);

    // move(40, 40, 1.5, 58, 15, 3);
    // turnGyro(75, 90);
    // setrobot_bw(2);
    // go_to_mission_on_stand('T', 17, 80);  // T คือเอามือทั้งสองเข้าไปวาง   5 คือ ระยะที่ยกแขนขึ้น
    // move(-40, -40, 1.5, 20, 0, 2);
    // setrobot_bw(2);

    // turnGyro(75, 90);
    // move(-40, -40, 1.5, 27, 0, 2);
    // setrobot_bw(2);

    // turnGyro(75, -90);
    // can_to_armL();  //เอากระป๋องจากที่ช่องซ้าย ไปให้มือซ้าย
    // can_to_armR();  //เอากระป๋องจากที่ช่องขวา ไปให้มือขวา
    // arm_start_new();
    // go_to_mission_on_stand('T', 13, 80);  // T คือเอามือทั้งสองเข้าไปวาง   5 คือ ระยะที่ยกแขนขึ้น
    // move(-40, -40, 1.5, 30, 0, 2);

    // turnGyro(75, 90);
    // setrobot_bw(2);
    // move(60, 60, 1.5, 58, 15, 3);
    // turnGyro(75, -90);

    // setrobot_bw(2);
    // move_bridge(40, 40, 1.5, 65, 15, 3);
    // turnGyro(75, 90);
    // setrobot_fw(2);
    // turnGyro(75, -90);

    // move_bridge(40, 40, 1.5, 50, 15, 3);
    // turnGyro(75, -90);
    // setrobot_bw(2);
    // turnGyro(75, 90);

    // move(60, 60, 1.5, 30, 15, 3);
    // turnGyro(75, 90);
    // move(-40, -40, 1.5, 58, 0, 2);
    // setrobot_bw(2);
  }