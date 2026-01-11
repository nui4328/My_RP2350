void readMe()
  {

        //คำสั่งพื้นฐานที่สำคัญ    ควบคุมมอเตอร์C++
    board.Motor(80, 80);    // เดินหน้า
    board.Motor(-80, -80);  // ถอยหลัง
    board.Motor(60, -60);   // หมุนขวา
    board.Motor(-60, 60);   // หมุนซ้าย
    board.Motor(0, 0);      // หยุด

      //ควบคุม Servo (ช่อง 16 17 18 )
    board.servo(16, 90);    // หมุนไป 90 องศา
    board.servoDetach(16);  // ปลดล็อก servo


    board.adcRead(0);
    board.adc_min(0);     // ค่า min ของเซ็นเซอร์ช่อง 0
    board.adc_max(0);     // ค่า max ของเซ็นเซอร์ช่อง 0
    //(board.adc_min(0) + board.adc_max(0)) / 2  // ค่ากลาง (thresh)
  }