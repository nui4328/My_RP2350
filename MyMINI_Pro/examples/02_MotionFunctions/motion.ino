
void _motion()
  {
      motor(30, 30); delay(500); motor(0, 0);
      servo(18, 90); servo(22, 45); servo(28, 120);
      servo(0, 90);  // Use only when UART0 TX is unused.
      servo(1, 90);  // Use only when UART0 RX is unused.

      f_line(40, 40, 0.85f, f3, 0);
      f_line(40, 40, 0.85f, 30.0f, 10);
      b_line(40, 40, 0.85f, b0, 0);
      b_line(40, 40, 0.85f, 30.0f, 10);

      fw_gyro(0.0f, 70, 0.80f, 40.0f, 10);
      bw_gyro(0.0f, 70, 0.80f, 30.0f, 10);

      turn(tfl, 60, f4, 5);  turn(tfr, 60, f11, 5);
      turn(tcl, 60, f4, 5);  turn(tcr, 60, f11, 5);
      turn(tl, 60, f4, 5);   turn(tr, 60, f11, 5);

      rotate_spin(90.0f, 60, 10);   // Relative angle.
      rotate_spin(-90.0f, 60, 10);
      rotateFW_pivot(60.0f, 60, 10);  // Absolute heading when gyro is ready.
      rotateFW_pivot(0.0f, 60, 10);
      rotateBW_pivot(60.0f, 60, 10);  // Absolute heading when gyro is ready.
      rotateBW_pivot(0.0f, 60, 10);
  }