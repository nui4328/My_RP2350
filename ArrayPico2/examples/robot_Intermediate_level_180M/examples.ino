void box1()
  {
    move_fw(40, 1.5, 30);
    move_fw(40, 1.5, 90, "line");
    turnGyro(50, 90);
    move_fw(40, 1.5, 60, "line");
    turnGyro(50, 90);
    set_b(2);
    move_fw(40, 1.5, 30, "none_line");
    turnGyro(50, 90);
    move_fw(40, 1.5, 30, "line");
    turnGyro(50, -90);
    set_b(2);

    move_fw(40, 1.5, 50, "line");
    release_box();     //  ปล่อยกล่อง
  }

void box2()
  {  
    set_f(2);
    delay(200);
    move_bw(40, 1.5, 17, "none_line");
    turnGyro(50, -90);
    set_b(2);

    move_fw(40, 1.5, 60, "none_line");
    turnGyro(50, 90);

    move_fw(40, 1.5, 30, "line");
    turnGyro(50, 90);

    move_fw(30, 1.5, 20, "line");
    release_box();     //  ปล่อยกล่อง
    
  }
void checkpoint_1()
  {  
    set_f(2);
    delay(200);
    move_bw(40, 1.5, 80, "line");
    turnGyro(50, 90);

    set_b(2);
    move_fw(40, 1.5, 90, "line");
    turnGyro(50, -90);
    set_b(2);
    move_fw(40, 1.5, 60, "line");    
  }
void box3()
  {  
    set_f(2);
    delay(200);
    move_bw(40, 1.5, 60, "line");
    turnGyro(50, -90);
    set_b(2);

    move_fw(40, 1.5, 30, "none_line");
    turnGyro(50, -90);
    move_fw(40, 1.5, 60, "line");
    turnGyro(50, -90);

    move_fw(40, 1.5, 30, "line");
    turnGyro(50, -90);
    set_b(2);

    move_fw(30, 1.5, 20, "line");
    release_box();     //  ปล่อยกล่อง
  }

void box4()
  {  
    set_f(2);
    delay(200);
    move_bw(40, 1.5, 20, "line");
    turnGyro(50, -90);
    set_b(3);

    move_fw(40, 1.5, 80, "line");
    release_box();     //  ปล่อยกล่อง
  }

void checkpoint_2()
  {  
    set_f(2);
    delay(200);
    move_bw(40, 1.5, 17, "none_line");
    turnGyro(50, 90);

    set_b(2);
    move_fw(40, 1.5, 30, "line");
    delay(200);    
  }

void go_home()
  {  
    set_f(2);
    delay(200);
    move_bw(40, 1.5, 30, "line");
    turnGyro(50, 90);
    
    move_fw(30, 1.5, 30, "none_line");
    turnGyro(50, -90);
    set_b(2);

    move_fw(30, 1.5, 60, "none_line");
    turnGyro(50, -90);

    move_fw(30, 1.5, 30, "none_line");
    turnGyro(50, 90);
    set_b(3);

    move_fw(40, 1.5, 110, "line");
    turnGyro(50, 90);

    move_fw(40, 1.5, 30, "line");
    turnGyro(50, 90);
    set_b(2);

    move_fw(40, 1.5, 30, "line");
    turnGyro(50, -90);

    move_fw(40, 1.5, 30, "line");
    turnGyro(50, -90);
    set_b(2);

    move_fw(40, 1.5, 60, "line");
    turnGyro(50, -90);
    set_b(2);    

    move_fw(40, 1.5, 80, "line");
    move_fw(40, 1.5, 20);
  }

