#include <MyMINI_Pro.h>

// Compile-only check for all six existing TurnMode aliases. Never called.
void compileAllTurnGyroModes() {
  turn_gyro(tfl, 60, -90.0f, 5);
  turn_gyro(tfr, 60,  90.0f, 5);
  turn_gyro(tcl, 60,   0.0f, 5);
  turn_gyro(tcr, 60,  90.0f, 5);
  turn_gyro(tl,  60, -90.0f, 5);
  turn_gyro(tr,  60,  90.0f, 5);
}

void setup() {}
void loop() {}
