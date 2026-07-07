
void hand_R_place_right()
  {
    int servo_LR = 70; // ค่าองศาของหัวไหล่ที่วางตรงกลงหลุม
    int space_LR = 15;
    arm_shoulder('R', servo_LR);  delay(200);
    arm_hand('R', 60);      delay(100);   //กางมือหลวมๆ
    for(int i = servo_LR; i >= servo_LR-space_LR ; i--)
          {arm_shoulder('R', i); delay(10);}

    for(int i=0; i<=3; i++)     //--- จำนวนครั้งที่โยก
      {
        for(int i = servo_LR-space_LR; i <= servo_LR+space_LR; i++)
          {arm_shoulder('R', i); delay(10);}
        for(int i = servo_LR+space_LR; i >= servo_LR-space_LR; i--)
          {arm_shoulder('R', i); delay(10);}
      }
    for(int i = servo_LR-space_LR; i <= servo_LR; i++)
        {arm_shoulder('R', i); delay(10);}
    
    arm_hand('R', 90);      delay(300);   //กางมือหลวมๆ
  }

void hand_R_place_middle()
  {
    int servo_LR = 30; // ค่าองศาของหัวไหล่ที่วางตรงกลงหลุม
    int space_LR = 15;
    arm_shoulder('R', servo_LR);  delay(200);
    arm_hand('R', 60);      delay(100);   //กางมือหลวมๆ
    for(int i = servo_LR; i >= servo_LR-space_LR ; i--)
          {arm_shoulder('R', i); delay(10);}

    for(int i=0; i<=3; i++)     //--- จำนวนครั้งที่โยก
      {
        for(int i = servo_LR-space_LR; i <= servo_LR+space_LR; i++)
          {arm_shoulder('R', i); delay(10);}
        for(int i = servo_LR+space_LR; i >= servo_LR-space_LR; i--)
          {arm_shoulder('R', i); delay(10);}
      }
    for(int i = servo_LR-space_LR; i <= servo_LR; i++)
        {arm_shoulder('R', i); delay(10);}
    
    arm_hand('R', 90);      delay(300);   //กางมือหลวมๆ
    
  }

////////////////////////////////////////////////////////////////////////หัวไหล่ซ้าย
void hand_L_place_left()
  {
    int servo_LR = 70; // ค่าองศาของหัวไหล่ที่วางตรงกลงหลุม
    int space_LR = 15;
    arm_shoulder('L', servo_LR);  delay(200);
    arm_hand('L', 60);      delay(100);   //กางมือหลวมๆ
    for(int i = servo_LR; i >= servo_LR-space_LR ; i--)
          {arm_shoulder('L', i); delay(10);}

    for(int i=0; i<=3; i++)     //--- จำนวนครั้งที่โยก
      {
        for(int i = servo_LR-space_LR; i <= servo_LR+space_LR; i++)
          {arm_shoulder('L', i); delay(10);}
        for(int i = servo_LR+space_LR; i >= servo_LR-space_LR; i--)
          {arm_shoulder('L', i); delay(10);}
      }
    for(int i = servo_LR-space_LR; i <= servo_LR; i++)
        {arm_shoulder('L', i); delay(10);}
    
    arm_hand('L', 90);      delay(300);   //กางมือหลวมๆ
  }

void hand_L_place_middle()
  {
    int servo_LR = 30; // ค่าองศาของหัวไหล่ที่วางตรงกลงหลุม
    int space_LR = 15;
    arm_shoulder('L', servo_LR);  delay(200);
    arm_hand('L', 60);      delay(100);   //กางมือหลวมๆ
    for(int i = servo_LR; i >= servo_LR-space_LR ; i--)
          {arm_shoulder('L', i); delay(10);}

    for(int i=0; i<=3; i++)     //--- จำนวนครั้งที่โยก
      {
        for(int i = servo_LR-space_LR; i <= servo_LR+space_LR; i++)
          {arm_shoulder('L', i); delay(10);}
        for(int i = servo_LR+space_LR; i >= servo_LR-space_LR; i--)
          {arm_shoulder('L', i); delay(10);}
      }
    for(int i = servo_LR-space_LR; i <= servo_LR; i++)
        {arm_shoulder('L', i); delay(10);}
    
    arm_hand('L', 90);      delay(300);   //กางมือหลวมๆ
    
  }

