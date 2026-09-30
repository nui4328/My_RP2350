#include <MyMINI_Pro.h>

void stopFor(uint32_t milliseconds) {
  motor(0, 0);  // PH/EN normal stop = brake; it does not put nSLEEP low.
  delay(milliseconds);
}

void setup() {
  Serial.begin(115200);

  // TB6612FNG remains the default unless this line is present.
  select_motor_driver(MotorDriverType::DRV8874);
  robot_begin();
  if (!motor_driver_ready()) {
    Serial.println(F("DRV8874 is not ready; see motor_driver_status()."));
    for (;;) delay(1000); }
  wait_button();
  f_line(40, 40, 2.45f, f15, 10);
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
