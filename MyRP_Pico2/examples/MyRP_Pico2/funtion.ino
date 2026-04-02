void set_motor()
  {
    set_Freq("Coreless_Motors");      // กำหนดความถี่ให้กับมอเตอร์เกาหลีหรือ มอเตอร์ Coreless ("Coreless_Motors")
      //set_Freq("DC_Motors");        // กำหนดความถี่ให้กับมอเตอร์ธรรมกา(DC_Motors)
    
    distance_scale_fw(1.00);       // ปรับค่าเพื่อให้ได้ระยะทางที่เป็นจริงสำหรับเดินหน้า
    distance_scale_bw(1.00);        // ปรับค่าเพื่อให้ได้ระยะทางที่เป็นจริงสำหรับถอยหลัง
   
    set_slow_motor(20, 20);     
    set_turn_center_l(-90, 90);
    set_turn_center_r(90, -90);
    set_turn_front_l(-30, 100);
    set_turn_front_r(100, -30);
    set_brake_fc(10, 30);
    set_brake_bc(10, 30);
    set_delay_f(10);
    
  }
