void gotoline()
  {
    // f_line(spl, spr, Kp, outloop_sensor หรือ distanceCm, offset)
    f_line(30, 30, 0.5f, f0,    1);  // เซนเซอร์หน้า F0      
    f_line(30, 30, 0.5f, cr,    1);  // เซนเซอร์กลางขวา CR
    f_line(30, 30, 0.5f, b0,    1);  // เซนเซอร์หลัง B0
    f_line(30, 30, 0.5f, 30.0f, 1);  // ระยะทาง 30 cm

    // b_line(spl, spr, Kp, outloop_sensor หรือ distanceCm, offset)
    b_line(30, 30, 0.5f, f0,    1);
    b_line(30, 30, 0.5f, cr,    1);
    b_line(30, 30, 0.5f, b0,    1);
    b_line(30, 30, 0.5f, 30.0f, 1);  // ถอย 30 cm
  }

void line_position_example()
  {
    positoin_error = 10;                 // ให้เส้นอยู่ฝั่งซ้ายของแถวเซนเซอร์
    f_line(30, 30, 1.5f, 20.0f, 1);   // จบแล้ว positoin_error กลับเป็น 50

    positoin_error = 90;                 // ให้เส้นอยู่ฝั่งขวาของแถวเซนเซอร์
    b_line(30, 30, 1.5f, 20.0f, 1);   // จบแล้ว positoin_error กลับเป็น 50
  }

void goto_gyro()
  {
    // deg, speed, Kp, distanceCm, offset
    fw_gyro(0.0f,   30, 1.2f, 30.0f, 1);           //  องศาที่หุ่นยนต์หันหัวไป     ความเร็ว     kp    ค่าเบรค 
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
    // mode, speed, stopSensor, reverseBrakeMs
    turn(tfl, 60, f4,  5);  // Forward Left    โหมด   ความเร็วในการหมุน   เซนเซอร์ที่แตะเส้น   เวลาเบรกย้อน (ms)
    turn(tfr, 60, f11, 5);  // Forward Right   โหมด   ความเร็วในการหมุน   เซนเซอร์ที่แตะเส้น   เวลาเบรกย้อน (ms)

    turn(tcl, 60, f4,  5);  // Center Left    โหมด   ความเร็วในการหมุน   เซนเซอร์ที่แตะเส้น   เวลาเบรกย้อน (ms)
    turn(tcr, 60, f11, 5);  // Center Right    โหมด   ความเร็วในการหมุน   เซนเซอร์ที่แตะเส้น   เวลาเบรกย้อน (ms)

    turn(tl,  60, f4,  5);  // Left       โหมด   ความเร็วในการหมุน   เซนเซอร์ที่แตะเส้น   เวลาเบรกย้อน (ms)
    turn(tr,  60, f11, 5);  // Right      โหมด   ความเร็วในการหมุน   เซนเซอร์ที่แตะเส้น   เวลาเบรกย้อน (ms)

    turn_gyro(tfr, 60, 90.0f, 20);     // ไปที่ +90° จากจุด start; เริ่มเบรกก่อนเป้าหมาย 20°
    turn_gyro(tfl, 60, -90.0f, 20);    // ไปที่ -90° จากจุด start; จาก +90° ต้องหมุนซ้าย 180°
    // ถ้าต้องการกลับไปทิศเริ่มต้นหลัง +90° ให้ใช้ turn_gyro(tfl, 60, 0.0f, 20);
  }

void rotate_gyro()
  {
    rotate_spin(50, 90.0f, 20);   // ไปที่หัว +90° จากจุด Start; เบรกสวน 20 ms
    rotate_spin(50, -90.0f, 20);  // ไปที่หัว -90° จากจุด Start ทางที่สั้นที่สุด; เบรกสวน 20 ms
    rotate_spin(50, 0.0f, 20);    // กลับไปที่หัว 0° จากจุด Start

    rotateFW_pivot(60, 90.0f, 10);  // ล้อวงนอกเดินหน้าไปที่ +90° จากจุด start; เบรกสวน 10 ms
    rotateFW_pivot(60, 0.0f, 10);  // กลับไป heading 0° จากจุด start; เบรกสวน 10 ms

    rotateBW_pivot(50, -90.0f, 20);  // ล้อวงนอกถอยหลังไปที่ -90° จากจุด start; เบรกสวน 20 ms
    rotateBW_pivot(50, 0.0f, 20);  // กลับไปที่ 0° จากจุด start; เบรกสวน 20 ms
  }

  void sensor()
    {
      readADC_F(channel);  minADC_F(channel);  maxADC_F(channel);
      readADC_B(channel);  minADC_B(channel);  maxADC_B(channel);
      readADC_CL();        minADC_CL();        maxADC_CL();
      readADC_CR();        minADC_CR();        maxADC_CR();
    }

void gyro_mini()
  {
    float yaw, rateX, rateY, rateZ;
    if (!gyro_read(yaw, rateX, rateY, rateZ)) return;
    Serial.print(F("Yaw before reset: "));
    Serial.println(yaw, 2);
    Serial.print(F("Gyro X/Y/Z (deg/s): "));
    Serial.print(rateX, 2); Serial.print('/');
    Serial.print(rateY, 2); Serial.print('/');
    Serial.println(rateZ, 2);
    if (!gyro_reset() || !gyro_read(yaw, rateX, rateY, rateZ)) return;
    Serial.print(F("Yaw after reset: "));
    Serial.println(yaw, 2);
  }
