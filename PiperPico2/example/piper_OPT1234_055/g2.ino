void mission_2() 
  {
    arm_updown(0);
    setrobot_bw(2);   //----->>ถอยหลังเซต; 

    move(40,  40, 1.5, 40, 10, 2);
    arm_ready();

    go_mission_on_stand(); //เข้าคีบบนแท่น 5 เซนติเมตร
    armLR_to_can();      //เอากระป๋องจากมือซ้ายไปเก็บที่เก็บกระป๋องซ้าย
    arm_updown(0);

    move(-40,  -40, 1.5, 15, 10, 2);

    turnGyro(70, -90, true);
    setrobot_bw(1);

    move(40,  40, 1.5, 30, 10, 2);

    turnGyro(70, -180, true);
    setrobot_bw(1);

    arm_ready();
    go_mission_on_floor();  // เข้าคีบที่พื้น

    move(-40,  -40, 1.5, 15, 0, 2);
    setrobot_bw(1);

    turnGyro(70, -90, true);
    move(40,  40, 1.5, 30, 10, 2);




    









    





   




    








  //   turnGyro(70, -90, true); 
  //   setrobot_bw(2);   //----->>ถอยหลังเซต; 
    
  //   move(40,  40, 1.5, 30, 10, 2);
  //   turnGyro(70, -180, true); 
  //   setrobot_bw(2);   //----->>ถอยหลังเซต; 

  //   arm_ready();
  //   go_mission_on_floor();  // เข้าคีบที่พื้น
    
  //   armLR_to_can();   //เอากระป๋องจากมือทั้ง 2 ข้าง ไปเก็บที่ช่องเก็บ 
    
    
  //   move(-40,  -40, 1.5, 30, 0, 2);
  //   setrobot_bw(2);   //----->>ถอยหลังเซต; 
  //   turnGyro(70, -90, true); 


  //   arm_ready();
  //   go_mission_on_floor();  // เข้าคีบที่พื้น

  //   arm_updown(7); 
  //   setrobot_fw(2);   //----->>ถอยหลังเซต; 
  //   turnGyro(70, 180, true); 
  //   setrobot_bw(2);   //----->>ถอยหลังเซต; 

  //   arm_updown(14); 
  //   go_to_mission_on_stand('L', arm_shoulderL_begin);  
  //   sw();
  //   move(-40,  -40, 1.5, 10, 10, 17);
  //   armR_to_armL();  //------->> เปลี่ยนกระป๋องจากมือ ขวา ไปมือ ซ้าย

  //   can_to_armR();    //เอากระป๋องจากที่ช่องขวา ไปให้มือขวา
  //   arm_updown(14); 
  //   go_to_mission_on_stand('R',  60);   // ตัวแปรตัวสุดท้ายคือ ความสูงของแขน





  //    sw();
    
  //  arm_bridge();

  //   move(40,  40, 1.5, 60, 0, 2);
  //   setrobot_fw(2);   //----->>ถอยหลังเซต; 
    
  //    turnGyro(70, -90, true); 
    
  //    move(-40,  -40, 1.5, 30, 0, 2);
  //    setrobot_bw(2);   //----->>ถอยหลังเซต; 
  //    turnGyro(70, 0, true); 
     
  //   move(-40,  -40, 1.5, 30, 0, 2);
  //   setrobot_bw(2);   //----->>ถอยหลังเซต; 

  //   turnGyro(70, -90, true); 
    

  //    move(60, 60, 1.5, 48, 0, 2);
  //   setrobot_fw(2);   //----->>ถอยหลังเซต; 
  //   turnGyro(70, -180, true); 
  //   setrobot_bw(2);   //----->>ถอยหลังเซต; 

  //   move_chopsticks(40,  40, 1.5, 60, 0, 2);

  //   move(40,  40, 1.5, 30, 20, 5);
  //   setrobot_fw(2);   //----->>ถอยหลังเซต; 
  //   turnGyro(70, 90, true); 
  //   arm_updown(0);


  //   move(40,  40, 1.5, 25, 10, 2);
  //   turnGyro(70, 0, true); 
  //   setrobot_bw(2);   //----->>ถอยหลังเซต;
  //   arm_ready();
  //   delay(300);
  //   turnGyro(70, 90, true);

  //   go_mission_on_stand(); //เข้าคีบบนแท่น 5 เซนติเมตร

  //    move(-40,  -40, 1.5, 37, 10, 2);
  //    turnGyro(70, 0, true); 
  //    setrobot_bw(2);   //----->>ถอยหลังเซต; 
    
  //   move(40,  40, 1.5, 30, 10, 2);
  //   turnGyro(70, 90, true); 

  //   move(40,  40, 1.5, 30, 10, 2);
  //   turnGyro(70, 0, true); 
  //    move(40,  40, 1.5, 30, 10, 2);
  //   setrobot_fw(2);   //----->>ถอยหลังเซต; 
  //   turnGyro(70, 90, true);
  //   arm_updown(19); 
  //   go_to_mission_on_stand('L',  70);   // ตัวแปรตัวสุดท้ายคือ ความสูงของแขน  เดินวางสีแดง
  //   move(-40,  -40, 1.5, 20, 10, 17);
  //   armR_to_armL();  //------->> เปลี่ยนกระป๋องจากมือ ขวา ไปมือ ซ้าย

  //   can_to_armR();    //เอากระป๋องจากที่ช่องขวา ไปให้มือขวา
  //    arm_updown(19); 
  //    go_to_mission_on_stand('R',  60);   // ตัวแปรตัวสุดท้ายคือ ความสูงของแขน
    







  }