#include <MyMINI_Pro.h>

void setup() {
  robot_begin();
  wait_button();

  
    fw_gyro(0.0f, 70, 0.80f, 15.0f, 1);
    fw_gyro(45.0f, 70, 0.80f, 40.0f, 1);
    fw_gyro(-45.0f, 70, 0.80f, 45.0f, 1);
    fw_gyro(0.0f, 70, 0.80f, 10.0f, 1);
    bw_gyro(0.0f, 70, 0.80f, 15.0f, 1);

    bw_gyro(-90.0f, 70, 0.80f, 88.0f, 1);
    bw_gyro(0.0f, 70, 0.80f, 19.0f, 1);
    bw_gyro(45.0f, 70, 0.80f, 40.0f, 1);
    bw_gyro(0.0f, 70, 0.80f, 15.0f, 1);

    fw_gyro(0.0f, 70, 0.80f, 10.0f, 1);
    fw_gyro(45.0f, 70, 0.80f, 40.0f, 1);
    fw_gyro(0.0f, 70, 0.80f, 40.0f, 1);

    bw_gyro(0.0f, 70, 0.80f, 80.0f, 1);

    fw_gyro(0.0f, 70, 0.80f, 55.0f, 1);
    fw_gyro(-90.0f, 70, 0.80f, 60.0f, 1);
    fw_gyro(0.0f, 70, 0.80f, 20.0f, 1);

    bw_gyro(0.0f, 70, 0.80f, 80.0f, 1);

    fw_gyro(0.0f, 70, 0.80f, 45.0f, 1);
    fw_gyro(45.0f, 70, 0.80f, 42.0f, 1);
    fw_gyro(0.0f, 70, 0.80f, 10.0f, 1);

    bw_gyro(0.0f, 70, 0.80f,5.0f, 1);
    bw_gyro(45.0f, 70, 0.80f, 85.0f, 1);
    bw_gyro(0.0f, 70, 0.80f, 25.0f, 1);







    // delay(500);
    // bw_gyro(0.0f, 70, 0.80f, 30.0f, 1);
}

void loop() {
  robot_update();
}
