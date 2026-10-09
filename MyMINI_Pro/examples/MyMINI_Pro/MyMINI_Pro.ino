#include <MyMINI_Pro.h>

void gyro_mini() {
  float yaw = 0.0f;
  float rateX = 0.0f;
  float rateY = 0.0f;
  float rateZ = 0.0f;
  if (!gyro_read(yaw, rateX, rateY, rateZ)) {
    Serial.println(F("gyro_mini: sensor read failed"));
    return;
  }
  Serial.print(F("gyro_mini yaw before reset: "));
  Serial.println(yaw, 2);
  Serial.print(F("gyro_mini rate x/y/z (deg/s): "));
  Serial.print(rateX, 2);
  Serial.print('/');
  Serial.print(rateY, 2);
  Serial.print('/');
  Serial.println(rateZ, 2);

  if (!gyro_reset() || !gyro_read(yaw, rateX, rateY, rateZ)) {
    Serial.println(F("gyro_mini: reset or verification failed"));
    return;
  }
  Serial.print(F("gyro_mini yaw after reset: "));
  Serial.println(yaw, 2);
}

void setup() {
  robot_setup();
  set_line_error_monitor(true);
  wait_button();
  gyro_set();

  rotateFW_pivot(60, 90.0f, 10);
  fw_gyro(0.0f, 70, 1.25f, 65.0f, 1);    delay(150);
  rotateFW_pivot(60, -90.0f, 10);
  fw_gyro(-90.0f, 70, 1.25f, 45.0f, 1);    delay(150);
  delay(1000);
  rotate_spin(70, 180.0f, 5);
  fw_gyro(180.0f, 70, 1.25f, 45.0f, 1);    delay(150);
  delay(1000);

  //  f_line(30, 30, 0.0f, 10, 0);
  //   f_line(60, 60, 2.0f, f0&15, 1);
  //   turn(tfr, 100, f10,  1);
    
  //   f_line(60, 60, 1.2f, 30,10);
  //   turn(tfr, 80, f12, 5);
  //   f_line(40, 40, 0.75f, f0, 2);
  //   turn(tcl, 80, f5,  2);

  //   f_line(60, 60, 1.2f, f0&15, 1);
  //   turn(tfr, 90, f10,  1);

  //   f_line(80, 80, 0.5f, f0, 1);
  //   turn(tcl, 80, f6,  1);

  //   f_line(70, 70, 0.5f, f1, 2);
  //   turn(tfl, 80, f5,  1);

  //   f_line(60, 60, 0.80f, 55, 0);
   
  //   f_line(50, 50, 0.80f, f15, 10);
  //   turn(tfr, 100, f12,  1);
    
  //   f_line(60, 60, 0.30f, f0, 5);
  //   turn(tcl, 50, f7, 5);
    
  //   f_line(30, 30, 0.45f, f15, 2);delay(150);

  //   gyro_reset();delay(100);
  //   rotateFW_pivot(50, 80.0f, 0);
  //   rotateFW_pivot(50, 10.0f, 0);
  //   fw_gyro(0.0f, 70, 1.25f, 65.0f, 1);    delay(150);
  //   bw_gyro(0.0f, 70, 0.65f, 55.0f, 1);  
  //   rotateBW_pivot(50, 80.0f, 00);
  //   rotateBW_pivot(50, 0.0f, 10);

  //   /////////////////////////////////////////////////////////box2
    
  //   b_line(30, 30, 0.5f, b15, 2);
  //   turn(tcl, 80, f7, 1);
    
  //   f_line(70, 70, 0.5f, f1,5);
  //   turn(tfl, 90, f6,  1);
    
  //    f_line(60, 60, 0.80f, 70, 2);
   
  //   // f_line(50, 50, 0.70f, f15, 2);
  //   turn(tfr, 80, f10,  1);

  //   f_line(70, 70, 0.5f, f1, 2);
  //   turn(tfr, 80, f10,  1);

  //   f_line(90, 90, 0.5f, cl, 0);
  //   f_line(90, 90, 0.5f, 20, 1);
  //   turn(tfl, 80, f10,  1);
    
  //   f_line(70, 70, 0.5f, 25, 1);
  //   turn(tfr, 90, f6,  1);

  //   f_line(80, 80, 2.5f, f0&15, 2); 


}

void loop() {
  //Serial.println(error, 2);
  delay(20);
  // motor(30, 30);    // Forward.
  // delay(700);
  // stopFor(400);

  // motor(-30, -30);  // Reverse.
  // delay(700);
  // stopFor(400);

  // motor(-30, 30);   // Turn left using the existing motor(left, right) contract.
  // delay(550);
  // stopFor(400);

  // motor(30, -30);   // Turn right.
  // delay(550);
  // stopFor(800);
}
