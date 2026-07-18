

void check_color_in2arm(int box)
  {
    if (tcs0.readFast()) 
      {
        Serial.print("TCS0: "); Serial.println(tcs0.getColor());
        color_armL = tcs0.getColor();
      }
    if (tcs1.readFast()) 
      {
        Serial.print("TCS1: "); Serial.println(tcs1.getColor());
        color_armR = tcs0.getColor();
      }
    
    ////-----------------------------------วางกระป๋อง
    if(color_armL == "RED" && color_armR == "GREEN")
      {
        arm_updown(3);
        arm_hand('L', 65);      // กางมือออก
        arm_hand('R', 65);      // กางมือออก
        arm_updown(0);
        arm_hand('L', 30);      // กางมือออก
        arm_hand('R', 30);      // กางมือออก
        delay(200);

        arm_updown(box);
        move_stuck(20, 1.5, 500); 
        hand_R_place_right();
        hand_L_place_left();
      }
    else if(color_armL == "RED" && color_armR == "YELLO")
      {
        arm_updown(3);
        arm_hand('L', 65);      // กางมือออก
        arm_hand('R', 65);      // กางมือออก
        arm_updown(0);
        arm_hand('L', 30);      // กางมือออก
        arm_hand('R', 30);      // กางมือออก
        delay(200);

        arm_updown(box);
        move_stuck(20, 1.5, 500); 
        hand_R_place_right();
        hand_L_place_middle();
      }
  }


