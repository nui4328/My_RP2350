
void servo_red()
  {   
    red_box = 1;
    robot.playTone(3000, 100);delay(100); 
    robot.ServoWrite(18, servo18-65);
    delay(800);
    robot.ServoWrite(18, servo18);
     //------------->>  ตรวจสอบนับการวางกล่องสี
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        robot.ServoWrite(16, servo16+120);    //-------------->>>  ยกธง
        robot.playTone(3000, 100);delay(100); 
        robot.playTone(3000, 100);delay(100); 
        robot.run();  // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN) 
      }
    delay(200);
  }

void servo_green()
  {
    green_box = 1;
    robot.playTone(3000, 100);delay(100); 
    robot.playTone(3000, 100);delay(100); 
    robot.ServoWrite(18, servo18+65);
    delay(800);
    robot.ServoWrite(18, servo18);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        robot.ServoWrite(16, servo16+120);   //-------------->>>  ยกธง
        robot.playTone(3000, 100);delay(100); 
        robot.playTone(3000, 100);delay(100); 
        robot.run();  // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN)  
      }
    delay(200);
  }

void servo_yellow()
  {
    yello_box = 1;
    robot.playTone(3000, 100);delay(100); 
    robot.playTone(3000, 100);delay(100); 
    robot.playTone(3000, 100);delay(100); 
    robot.playTone(3000, 100);delay(100); 
    robot.ServoWrite(17, servo17+65);
    delay(800);
    robot.ServoWrite(17, servo17);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        robot.ServoWrite(16, servo16+120);     //-------------->>>  ยกธง
        robot.playTone(3000, 100);delay(100); 
        robot.playTone(3000, 100);delay(100); 
        robot.run(); // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN)     
      }
    delay(200);
  }
void servo_blue()
  {
    blue_box = 1;
    robot.playTone(3000, 100);delay(100); 
    robot.playTone(3000, 100);delay(100); 
    robot.playTone(3000, 100);delay(100); 
    robot.ServoWrite(17, servo17-65);
    delay(800);
    robot.ServoWrite(17, servo17);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        robot.ServoWrite(16, servo16+120);         //-------------->>>  ยกธง 
        robot.playTone(3000, 100);delay(100); 
        robot.playTone(3000, 100);delay(100); 
        robot.run(); // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN)       
      }
     delay(200);
  }
