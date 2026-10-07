#include <MyMINI_Pro.h>


void setup() {
  robot_setup();
  set_line_error_monitor(true);
  wait_button();
  if (!gyro_start_ready() && !gyro_recover()) {
    Serial.println(F("Gyro not ready; mission stopped."));
    motor(1, 1);
    return;
  }

    f_line(30, 30, 0.0f, 10, 0);
    f_line(60, 60, 2.0f, f0&15, 1);
    turn(tfr, 80, f11,  0);
    
    f_line(60, 60, 1.2f, 30, 5);
    turn_gyro(tfr, 70, 170.0f,  5);
    f_line(40, 40, 0.75f, cl, 2);
    turn(tl, 80, f5,  0);

    f_line(60, 60, 1.2f, f0&15, 1);
    turn(tfr, 80, f12,  0);

    f_line(80, 80, 0.5f, f0, 1);
    turn(tcl, 80, f6,  1);

    f_line(70, 70, 0.5f, f1, 2);
    turn(tfl, 80, f5,  0);

 f_line(50, 50, 0.45f, 35, 2);
   
    f_line(55, 45, 0.40f, f15, 2);
    turn(tfr, 80, f10,  0);
    
    f_line(60, 60, 0.30f, f0, 5);
    turn(tcl, 50, f5, 5);
    
    f_line(30, 30, 0.45f, f15, 2);delay(150);

    rotateFW_pivot(80, 70.0f, 2);

    fw_gyro(0.0f, 85, 1.75f, 65.0f, 1);    delay(150);
    bw_gyro(0.0f, 85, 0.65f, 55.0f, 1);  


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
