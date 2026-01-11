

void set_MMEspMini()
  {
    Serial.begin(115200);

    board.begin();
    ui.begin();

    encoder.setupEncoder();      // เรียก setupEncoder() หลังสร้าง object
    encoder.resetEncoders();

    Wire.begin(15, 16);  // SDA=15, SCL=16
    Wire1.begin(7, 8);
    gyro.begin();
    gyro.resetAngles();
    board.loadCalibrationFromEEPROM();

    ui.showWelcomeScreen();
    ui.showMainMenu();
    ui.waitForButtonAction();
  }

 