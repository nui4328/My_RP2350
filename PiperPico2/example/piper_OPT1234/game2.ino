void mission2()
{
    arm_updown(0);
    move(40,  40, 1.5, 30, 15, 1);               //---------เดินไปหยุดที่บล๊อก             
    turnGyro(70, -90); 
    setrobot_bw(2);   //----->>ถอยหลังเซต
    move(40,  40, 1.5, 27, 5, 1);               //---------เดินไปหยุดที่บล๊อก             
    turnGyro(70, -90); 
    setrobot_bw(1);   //----->>ถอยหลังเซต


    arm_ready();       // เตียมมือจับตอนเข้าคีบ
    go_to_mission_to_can(0, 7);  // 0 คือ ระยะยกขึ้น  0 กระป๋องวางที่พื้น     , ระยะ 7



    move(-40,  -40, 1.5, 25, 0, 7);               //---------เดินไปหยุดที่บล๊อก  


    armLR_to_can();   //เอากระป๋องจากมือทั้ง 2 ข้าง ไปเก็บที่ช่องเก็บ 

    setrobot_bw(1);

    turnGyro(70, 90); 
 
    move(40,  40, 1.5, 27, 15, 3);               //---------เดินไปหยุดที่บล๊อก             
    turnGyro(70, -90); 
    setrobot_bw(1);

    arm_bridge();

    move_bridge(40,  40, 1.5, 60, 0, 3);

    setrobot_fw(1);
    turnGyro(75, -90); 
    

    move_bridge(40, 40, 1.5, 55, 0, 3);

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

    arm_updown(7);
    move(30,  30, 1.5, 28, 0, 7);
    setrobot_fw(1);

    turnGyro(75, -90); 

    arm_updown(0);

    move(30,  30, 1.5, 30, 0, 1);
    turnGyro(75, 90);
    setrobot_bw(1);
    turnGyro(75, -90); 

    arm_ready();
    go_mission_on_stand(); //เข้าคีบบนแท่น 5 เซนติเมตร

    move(-30,  -30, 1.5, 15, 0, 5);
    move(-30,  -30, 1.5, 40, 0, 3);

    turnGyro(75, -90);
    setrobot_bw(1);

    move(30,  30, 1.5, 30, 15, 3);
    turnGyro(75, 90);

    move(30,  30, 1.5, 30, 15, 3);
    turnGyro(75, -90);
    setrobot_bw(1);

    move(30,  30, 1.5, 30, 0, 3);
    setrobot_fw(1);

    turnGyro(75, 90);
    setrobot_bw(1);

    go_to_mission_on_stand('L',  19, 85);    // //  วางบนแท่น 10 cm
    move(-30,  -30, 1.5, 15, 0, 19);

    armR_to_armL();
    can_to_armR();

    go_to_mission_on_stand('R',  20, 85);    // //  วางบนแท่น 10 cm
    move(-30,  -30, 1.5, 15, 0, 20);

    armL_to_armR();
    armR_to_can();

    setrobot_bw(1);
    arm_updown(3);

    turnGyro(75, 90);
    setrobot_bw(1);

    move(30,  30, 1.5, 30, 0, 3);
    setrobot_fw(1);

    turnGyro(75, -90);

    move(-60,  -60, 1.5, 70, 0, 3);
    setrobot_bw(1);

    turnGyro(75, -90);

    move(60,  60, 1.5, 50, 0, 3);

    turnGyro(75, 90);
    setrobot_bw(1);
    turnGyro(75, -90);

    arm_ready();
    move(30,  30, 1.5, 15, 0, 1);
    go_to_mission_to_can(0, 6);  // 0 คือ ระยะยกขึ้น  0 กระป๋องวางที่พื้น     , ระยะ 7

    move(-60,  -60, 1.5, 120, 0, 3);
    setrobot_bw(1);

    turnGyro(75, 90);
    setrobot_bw(1);
    
    move(30,  30, 1.5, 30, 15, 3);
    turnGyro(75, -90);

    move(-30,  -30, 1.5, 30, 15, 3);
    setrobot_bw(1);

    turnGyro(75, -90);



    go_to_mission_on_stand('L',  9, 60);    // R คือเอามือขวาเข้าไปวาง   5 คือ ระยะที่ยกแขนขึ้น 60 องศาวาง
    move(-40, -40, 1.5, 15, 0, 9);























    
















  







  






   
  
  
  
  
  







  
  













  
  

  

} 