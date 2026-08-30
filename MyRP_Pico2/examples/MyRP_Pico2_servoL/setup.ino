void set_motor()
  {    
    robot.set_Freq("Coreless_Motors");      // หรือ "DC_Motors"
    robot.sp_factor_cm_s_forFW(5.75);  
    robot.sp_factor_cm_s_forBW(5.75);
    
    
    robot.set_slow_motor(20, 20);     
    robot.set_turn_center_l(-90, 90);
    robot.set_turn_center_r(90, -90);
    robot.set_turn_front_l(-25, 90);
    robot.set_turn_front_r(90, -25);
    robot.set_brake_fc(10, 50);
    robot.set_brake_bc(10, 50);
    robot.set_delay_f(10);



    
  }