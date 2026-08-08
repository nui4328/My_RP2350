void set_motor()
  {    
    robot.set_Freq("Coreless_Motors");      // หรือ "DC_Motors"
    robot.sp_factor_cm_s_forFW(5.75);  
    robot.sp_factor_cm_s_forBW(5.75);
    
    
    robot.set_slow_motor(5, 5);     
    robot.set_turn_center_l(-40, 40);
    robot.set_turn_center_r(40, -40);
    robot.set_turn_front_l(-5, 60);
    robot.set_turn_front_r(60, -5);
    robot.set_brake_fc(10, 30);
    robot.set_brake_bc(10, 30);
    robot.set_delay_f(10);



    
  }