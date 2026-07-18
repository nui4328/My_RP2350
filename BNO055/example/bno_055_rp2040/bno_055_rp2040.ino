#include <Wire.h>
#include "BNO055.h"

BNO055 imu;

void setup() {
    Serial.begin(115200);

    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();
    Wire.setClock(400000);

    if (!imu.begin(0x29, Wire)) {
        Serial.println("BNO055 not found");
        while (1);
    }

    imu.setLPF(0.75f);
    imu.calibrate(11, false);
    imu.resetAngles();

    Serial.println("BNO055 Ready");
}

void loop() {
    imu.update();

    Serial.printf(
        "Yaw: %.2f  GyroZ: %.2f\n",
        imu.yaw(),
        imu.gyro('z')
    );

    delay(10);
}