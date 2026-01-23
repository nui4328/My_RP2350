void box1()
  {
    move_fw(40, 1.5, 60, "none_line");
    turnGyro(50, -90);
    set_b(2);
     move_fw(40, 1.5, 60, "none_line");
    turnGyro(50, 90);
     move_fw(40, 1.5, 60, "none_line");
    turnGyro(50, 90);
    set_b(3);
    
    move_fw(40, 1.5, 80, "line");
    set_f(3);
    release_box();              //  ปล่อยกล่อง
  }

void box2()
  {  
    move_bw(40, 1.5, 60, "line");
    turnGyro(50, 90);

    move_fw(40, 1.5, 60, "none_line");
    turnGyro(50, -90);
    set_b(2);
     move_fw(40, 1.5, 30, "none_line");
    turnGyro(50, -90);
     move_fw(40, 1.5, 10, "line");
    set_f(2); 
    release_box();              //  ปล่อยกล่อง
  }
void checkpoint_1()
  {  
    set_f(2);
    move_bw(40, 1.5, 20, "none_line");
    turnGyro(50, 90);

    move_fw(40, 1.5, 30, "line");
    set_f(2);
    turnGyro(50, -90);
    move_fw(40, 1.5, 30, "line");    
  }
void box3()
  {  
    set_f(2);
    move_bw(40, 1.5, 30, "none_line");
    turnGyro(50, -90);

    move_fw(40, 1.5, 30, "none_line");
    turnGyro(50, -90);
     move_fw(40, 1.5, 20, "line");
    release_box();              //  ปล่อยกล่อง
     set_f(2);
     move_bw(40, 1.5, 20, "none_line");
     turnGyro(50, -90);
     move_fw(40, 1.5, 30, "none_line");
    set_f(2); 
    turnGyro(50, 90);
    move_fw(40, 1.5, 60, "line");
  }