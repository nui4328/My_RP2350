void Order1C1() // จากจุดสตาท function นี้ไม่เหมือนเพื่อน
  {
    griper_set();
    f_line(50, 50, 0.4f, f0, 1); turn(tfl, 100, f6, 5);
    f_line(50, 50, 0.4f, f0, 1); turn(tfr, 100, f10, 5);
    f_line(50, 50, 0.4f, f15, 0);
    f_line(50, 50, 0.4f, cr, 0);
    f_line(50, 50, 0.4f, f15, 0);
    f_line(30, 30, 0.4f, cr, 20);
    // check_colorLEGO();
    f_line(50, 50, 0.4f, f15, 0);
    f_line(30, 30, 0.4f, b0, 20);
    motor(40, 1);
    delay(350);
    motor(-40, 1);
    delay(10);
    motor(1, 1);
    f_line(30, 30, 0.0f, f0, 1); turn(tfl, 100, f6, 5);
    f_line(20, 20, 0.2f, cl, 20);
    getCan();
    turn(tr, 40, f11, 10);
    f_line(40, 40, 0.4f, f15, 1); turn(tcr, 40, f11, 10);
  }
void Order2C1()
  {
    griper_set();
    f_line(50, 50, 0.4f, f0, 0);
    f_line(30, 30, 0.4f, cl, 20);
    getCan();
    turn(tr, 40, f11, 10);
    turn(tr, 40, f11, 10);
  }
void Order3C1()
  {
    griper_set();
    f_line(30, 30, 0.4f, b0, 20);
    motor(40, 1);
    delay(350);
    motor(-40, 1);
    delay(10);
    motor(1, 1);
    f_line(30, 30, 0.0f, f0, 1); turn(tfl, 100, f6, 5);
    f_line(20, 20, 0.2f, cl, 20);
    getCan();
    turn(tl, 40, f5, 10);
    f_line(50, 50, 0.4f, f0, 1); turn(tcl, 40, f5, 10);
  }
void Order1C2()
  {
    griper_set();
    f_line(50, 50, 0.4f, f0, 1); turn(tfl, 100, f6, 5);
    f_line(50, 50, 0.4f, f15, 1); turn(tfr, 100, f10, 5);
    f_line(50, 50, 0.4f, 20, 0);
    f_line(40, 40, 0.4f, f0, 0);
    f_line(30, 30, 0.4f, cl, 20);
    getCan();
    turn(tr, 40, f11, 10);
    f_line(50, 50, 0.4f, f15, 1); turn(tfr, 100, f10, 5);
    f_line(50, 50, 0.4f, 20, 0);
    f_line(40, 40, 0.4f, f0, 0);
    f_line(30, 30, 0.4f, cl, 10);
  }
void Order2C2()
  {
    griper_set();
    f_line(50, 50, 0.4f, f0, 0);
    f_line(50, 50, 0.4f, cl, 0);
    f_line(50, 50, 0.4f, 20, 0);
    f_line(40, 40, 0.4f, f0, 0);
    f_line(30, 30, 0.4f, cl, 20);
    getCan();
    turn(tr, 40, f11, 10);
    turn(tr, 40, f11, 10);
    f_line(50, 50, 0.4f, 20, 0);
    f_line(40, 40, 0.4f, f0, 0);
    f_line(30, 30, 0.4f, cl, 10);
  }
void Order3C2()
  {
    griper_set();
    f_line(50, 50, 0.4f, f15, 1); turn(tfr, 100, f10, 5);
    f_line(50, 50, 0.4f, f0, 1); turn(tfl, 100, f6, 5);
    f_line(50, 50, 0.4f, 20, 0);
    f_line(40, 40, 0.4f, f0, 0);
    f_line(30, 30, 0.4f, cl, 20);
    getCan();
    turn(tl, 40, f5, 10);
    f_line(40, 40, 0.4f, f0, 1); turn(tfl, 100, f6, 5);
    f_line(50, 50, 0.4f, 20, 0);
    f_line(40, 40, 0.4f, f0, 0);
    f_line(30, 30, 0.4f, cl, 10);
  }