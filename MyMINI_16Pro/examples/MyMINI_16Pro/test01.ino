void start_robot()
    {
        f_line(30, 30, 0.0f, 5.0f, 1);  // ระยะทาง 30 cm
    }
void goTo_t1()
    {
        f_line(30, 30, 0.5f, f15,    10);  // เซนเซอร์หน้า F0
        turn(tcr, 50, f10,  5);
        arm_downOpen();
        f_line(50, 50, 0.85f, 50.0f, 0);  // ระยะทาง 30 cm
        f_line(20, 20, 0.5f, 5.0f, 1);  // ระยะทาง 30 cm
        arm_downclose();
    }
void T1_to_Z3()
    {
        f_line(50, 50, 0.85f, f0,    10);  // เซนเซอร์หน้า F0
        turn(tcl, 40, f7,  10);
        f_line(30, 30, 0.5f, cr,    10); 

        rotateFW_pivot(40, 65.0f, 10); 
        fw_gyro(0.0f,   60, 1.2f, 50.0f, 1); 
        arm_downOpen(); 
    }