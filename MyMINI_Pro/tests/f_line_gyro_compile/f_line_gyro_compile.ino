#include <MyMINI_Pro.h>

// Compile both overloads: wrapped gyro target and existing float distance.
void compileFLineOverloads() {
  f_line(50, 50, 1.5f, gyro_angle(135.0f), 1);
  f_line(50, 50, 1.5f, gyro_angle_raw(495.0f), 1);
  f_line(50, 50, 1.5f, 20.0f, 1);
}

void setup() {}
void loop() {}
