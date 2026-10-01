#include <MyMINI_Pro.h>


void setup() {
  robot_setup();
  wait_button();

 
  
  f_line(30, 30, 1.5f, f15, 1);
  turn_gyro(tcr, 40, 70.0f, 20);
  //turn_gyro(tfl, 60, 0.0f, 5);   // กลับทิศ START


}

void loop() {
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
