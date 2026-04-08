void set_motor()
  {    
    robot.set_Freq("Coreless_Motors");      // หรือ "DC_Motors"
    robot.sp_factor_cm_s_forFW(1.75);  
    robot.sp_factor_cm_s_forBW(1.75);
    
    
    robot.set_slow_motor(20, 20);     
    robot.set_turn_center_l(-90, 90);
    robot.set_turn_center_r(90, -90);
    robot.set_turn_front_l(-30, 100);
    robot.set_turn_front_r(100, -30);
    robot.set_brake_fc(10, 30);
    robot.set_brake_bc(10, 30);
    robot.set_delay_f(10);
    
  }