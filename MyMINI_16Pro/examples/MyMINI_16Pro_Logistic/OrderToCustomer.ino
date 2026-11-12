void GC1()
  {
    f_line(50, 50, 0.4f, f0, 0);
    f_line(50, 50, 0.4f, f0, 0);
    f_line(40, 40, 0.4f, f0, 1); turn(tfl, 25, f9, 15);
    // f_line(10, 10, 0.4f, 3, 10);
    slide_drop();
    if(lastCan == 0)
      {
         b_line(30, 30, 0.4f, cl&cr, 10);turn(tl, 60, f4, 10);
        f_line(50, 50, 0.4f, f0, 0);
        f_line(50, 50, 0.4f, f0, 0);
        f_line(50, 50, 0.4f, cl, 10);
      }
    C1 = 1;
  }
void GC2()
  {
    f_line(50, 50, 0.4f, f0, 0);
    f_line(50, 50, 0.4f, f0, 0);
    f_line(40, 40, 0.4f, f0, 1); turn(tfr, 25, f6, 15);
    // f_line(10, 10, 0.4f, 5, 10);
    slide_drop();
    if(lastCan == 0)
      {
         b_line(30, 30, 0.4f, cl&cr, 10); turn(tr, 40, f11, 10);
        f_line(50, 50, 0.4f, f0, 0);
        f_line(50, 50, 0.4f, f0, 0);
        f_line(50, 50, 0.4f, cl, 10);
      }
    
    C2 = 1;
  }
void GC3()
  {
    f_line(50, 50, 0.4f, 40, 0);
    f_line(20, 20, 0.4f, f0, 1); turn(tfl, 25, f9, 15);
    // f_line(10, 10, 0.4f, 3, 10);
    slide_drop();
    if(lastCan == 0)
      {
        b_line(30, 30, 0.4f, cl&cr, 10); turn(tl, 40, f5, 10);
        f_line(50, 50, 0.4f, f0, 0);
        f_line(50, 50, 0.4f, cl, 10);
      }
    
    C3 = 1;
  }
void GC4()
  {
    f_line(50, 50, 0.4f, f0, 0);
    f_line(40, 40, 0.4f, f0, 1); turn(tfr, 25, f6, 15);
    // f_line(10, 10, 0.4f, 5, 10);
    slide_drop();
    if(lastCan == 0)
      {
         b_line(30, 30, 0.4f, cl&cr, 10); turn(tr, 40, f11, 10);
        f_line(50, 50, 0.4f, f0, 0);
        f_line(50, 50, 0.4f, cl, 10);
      }
    
    C4 = 1;
  }
void GC5()
  {
    f_line(40, 40, 0.4f, f0, 1); turn(tfl, 25, f9, 15);
    // f_line(10, 10, 0.4f, 3, 10);
    slide_drop();
    if(lastCan == 0)
      {
        b_line(30, 30, 0.4f, cl&cr, 10); turn(tl, 40, f5, 10);
      }
    C5 = 1;
  }
void GC6()
  {
    f_line(40, 40, 0.4f, f0, 1); turn(tfr, 25, f6, 15);
    // f_line(10, 10, 0.4f, 5, 10);
    slide_drop();
    if(lastCan == 0)
      {
         b_line(30, 30, 0.4f, cl&cr, 10); turn(tr, 40, f11, 10);
      }
    C6 = 1;
  }