void C1_T_C2()
  {
    b_line(30, 30, 0.4f, cl, 10); turn(tr, 40, f11, 10);
    turn(tr, 40, f11, 10);
    f_line(10, 10, 0.4f, 5, 10);
    slide_down(200);
    delay(200);
    servo(18, 140);
    delay(200);
    slide_up(700);
    b_line(20, 20, 0.4f, cl, 10); turn(tr, 40, f11, 10);
    f_line(50, 50, 0.4f, f0, 0);
    f_line(50, 50, 0.4f, f0, 0);
    f_line(40, 40, 0.4f, cl, 10);
    slide_set();
    slide_up(700);
    emptyOrder();
    GC2();
  }
