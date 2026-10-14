void point_3_ring()
  {
     
      gyro_reset();
      rotateFW_pivot(40, 50.0f, 20); 
      rotateFW_pivot(40, 5.0f, 20); 
       f_line(40, 40, 0.0f, f8,    0);
       turn(tfr, 60, f6,  8); 
       positoin_error = 80;  
      f_line(70,70, 1.1f, gyro_angle(-135.0f), 20);
      delay(100);
      //if (!f_line_gyro_succeeded()) { motor(1, 1); return; }
      rotateFW_pivot(40, -250.0f, 20);  
      arm_minOpenUp();
      fw_gyro(-250.0f, 30, 1.2f, 10.0f, 1);
      delay(100); 
      bw_gyro(-250.0f, 50, 1.2f, 10.0f, 1);    //  ถอยหลังออกมาจากวงกลมเล็ก

      b_line(20, 20, 0.0f, cl,    2);
      turn(tl, 60, f3,  20);
      arm_closeDown();
      positoin_error = 20;  
      f_line(60, 60, 1.25f, gyro_angle(200.0f), 20);
      //if (!f_line_gyro_succeeded()) { motor(1, 1); return; }

      fw_gyro(180.0f, 60, 1.2f, 15.0f, 0);    ///----------------->>ระยะวิ่งออกจากวงกลม
      
      f_line(40, 40, 0.0f, f15, 1);
      turn(tcl, 50, b7,  5); 
      //f_line(30, 30, 0.5f, f15, 10);
      b_line(20, 20, 0.025f, cr, 20);
      turn(tr, 50, f9,  15); 
    
  }

void Zone_3_yollow()
  {
      f_line(15, 15, 0.5f, f0,    1);
      gyro_reset();
      rotateFW_pivot(40, 90.0f, 20); 
      rotateFW_pivot(40, 0.0f, 20); 
      fw_gyro(0.0f,   70, 1.2f, 60.0f, 1);
      delay(200);
      servoL(servo_L-90);
      servoR(servo_R-90);
      delay(300);
      servoUpDown(servo_down+90);
      delay(100);

      bw_gyro(0.0f,   70, 1.2f, 50.0f, 1);
      turn(tcl, 40, f6,  5);
      f_line(30, 30, 1.75f, cl,    2);
      turn(tl, 60, f6,  15);
       
  }
  
void Zone_3_toBox_L()
  {
      f_line(15, 15, 0.5f, f0,    1);
      gyro_reset();
      rotateFW_pivot(40, 90.0f, 20); 
      rotateFW_pivot(40, 0.0f, 20); 
      fw_gyro(0.0f,   70, 1.2f, 60.0f, 1);
      delay(200);
      servoL(servo_L-90);
      servoR(servo_R-90);
      delay(300);
      servoUpDown(servo_down+90);
      delay(100);

      bw_gyro(0.0f,   70, 1.2f, 50.0f, 1);
      f_line(20, 20, 0.0f, cl,    2);
      rotate_spin(50, -90.0f, 20);
      servoUpDown(servo_down);

      fw_gyro(-90.0f, 15, 1.2f, 23.0f, 1);
      delay(100);
      arm_closeDown();
      rotate_spin(30, 180.0f, 20);

      f_line(30, 30, 0.0f, f0,    5);
      turn(tcr, 50, b6,  5);
      b_line(20, 20, 0.75f, cl,    2);
      turn(tl, 40, f6,  5);     
  }