// Reorder destinations are still being implemented (reOrder1..6 are absent).
// Keep this inactive until those motion functions exist.
#if 0
void check_re_order()
  {
    if(_bic_color == order_color[0])
      {
        reOrder1();
      }
    else if(_bic_color == order_color[1])
      {
        reOrder2();
      }
    else if(_bic_color == order_color[2])
      {
        reOrder3();
      }
    else if(_bic_color == order_color[3])
      {
        reOrder4();
      }
    else if(_bic_color == order_color[4])
      {
        reOrder5();
      }
    else if(_bic_color == order_color[5])
      {
        reOrder6();
      }
  }
#endif

void re_order()
  {
    if(_can_color == order_color[0])
      {
        GC1();
      }
    else if(_can_color == order_color[1])
      {
        GC2();
      }
    else if(_can_color == order_color[2])
      {
        GC3();
      }
    else if(_can_color == order_color[3])
      {
        GC4();
      }
    else if(_can_color == order_color[4])
      {
        GC5();
      }
    else if(_can_color == order_color[5])
      {
        GC6();
      }
  }
