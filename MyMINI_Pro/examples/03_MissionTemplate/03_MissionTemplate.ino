#include <MyMINI_Pro.h>

void setup() {
  robot_begin();
  wait_button();

  set_turn_overshoot(20);
  set_turn_timeout(3000);
  set_turn_approach_kp(0.15f);
  set_turn_approach(10, 30, 15);
  set_turn_line_search(70, 60);
  set_rotate_fallback(300, 550, 50);

  // Write the mission sequence here.
  // f_line(60, 60, 0.85f, f0, 0);
  // turn(tcr, 60, f11, 5);
  // fw_gyro(0.0f, 70, 0.80f, 40.0f, 5);
  // rotateBW_pivot(60.0f, 60, 5);
  // rotateBW_pivot(0.0f, 60, 5);
}

void loop() {
  robot_update();
}
