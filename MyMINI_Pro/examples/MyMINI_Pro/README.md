void gotoline()
  {
    // f_line(spl, spr, Kp, outloop_sensor หรือ distanceCm, offset)
    f_line(30, 30, 1.5f, f0,    1);  // เซนเซอร์หน้า F0
    f_line(30, 30, 1.5f, cr,    1);  // เซนเซอร์กลางขวา CR
    f_line(30, 30, 1.5f, b0,    1);  // เซนเซอร์หลัง B0
    f_line(30, 30, 1.5f, 30.0f, 1);  // ระยะทาง 30 cm

    // b_line(spl, spr, Kp, outloop_sensor หรือ distanceCm, offset)
    b_line(30, 30, 1.5f, f0,    1);
    b_line(30, 30, 1.5f, cr,    1);
    b_line(30, 30, 1.5f, b0,    1);
    b_line(30, 30, 1.5f, 30.0f, 1);  // ถอย 30 cm
  }
void goto_gyro()
  {
    // deg, speed, Kp, distanceCm, offset
    fw_gyro(0.0f,   30, 1.2f, 30.0f, 1);
    fw_gyro(90.0f,  30, 1.2f, 30.0f, 1);
    bw_gyro(0.0f,   30, 1.2f, 30.0f, 1);
    bw_gyro(-90.0f, 30, 1.2f, 30.0f, 1);
  }
void motor_servo()
  {
    motor(30, 30);     // เดินหน้า
    motor(-30, -30);   // ถอยหลัง
    motor(-30, 30);    // หมุนซ้าย
    motor(30, -30);    // หมุนขวา
    motor(0, 0);       // coast: ปล่อยล้อ
    motor(1, 1);       // เบรกสองล้อ
    motor(-1, -1);     // เบรกสองล้อเช่นกัน

    servo(18, 90);
    servo(22, 45);
    servo(28, 120);
    // servo(0, 90);   // ใช้ได้เมื่อไม่ได้ใช้ UART0 TX
    // servo(1, 90);   // ใช้ได้เมื่อไม่ได้ใช้ UART0 RX
  }

void turn_online()
  {
    // mode, speed, stopSensor, stopPull
    turn(tfl, 60, f4,  5);  // Forward Left
    turn(tfr, 60, f11, 5);  // Forward Right
    turn(tcl, 60, f4,  5);  // Center Left
    turn(tcr, 60, f11, 5);  // Center Right
    turn(tl,  60, f4,  5);  // Left
    turn(tr,  60, f11, 5);  // Right

    urn_gyro(tfr, 60, 90.0f, 20);
    urn_gyro(tfr, 60, -90.0f, 20);
  }

void rotate_gyro()
  {
    rotate_spin(90.0f,    60, 10);  // หมุนขวา 90° จากมุมปัจจุบัน
    rotate_spin(-90.0f,   60, 10);  // หมุนซ้าย 90° จากมุมปัจจุบัน

    rotateFW_pivot(90.0f, 60, 10);  // ล้อวงนอกเดินหน้า ไป heading 90°
    rotateFW_pivot(0.0f,  60, 10);  // กลับไป heading 0°

    rotateBW_pivot(90.0f, 60, 10);  // ล้อวงนอกถอยหลัง ไป heading 90°
    rotateBW_pivot(0.0f,  60, 10);  // กลับไป heading 0°
  }

  void sensor()
    {
      readADC_F(channel);  minADC_F(channel);  maxADC_F(channel);
      readADC_B(channel);  minADC_B(channel);  maxADC_B(channel);
      readADC_CL();        minADC_CL();        maxADC_CL();
      readADC_CR();        minADC_CR();        maxADC_CR();
    }