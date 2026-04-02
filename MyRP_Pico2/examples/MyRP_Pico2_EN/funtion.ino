void set_motor()
  {
    set_Freq("Coreless_Motors");      // กำหนดความถี่ให้กับมอเตอร์เกาหลีหรือ มอเตอร์ Coreless ("Coreless_Motors")
      //set_Freq("DC_Motors");        // กำหนดความถี่ให้กับมอเตอร์ธรรมกา(DC_Motors)
    
    distance_scale_fw(1.00);       // ปรับค่าเพื่อให้ได้ระยะทางที่เป็นจริงสำหรับเดินหน้า
    distance_scale_bw(1.00);        // ปรับค่าเพื่อให้ได้ระยะทางที่เป็นจริงสำหรับถอยหลัง
   
    set_slow_motor(20, 20);     
    set_turn_center_l(-90, 90);
    set_turn_center_r(90, -90);
    set_turn_front_l(-30, 100);
    set_turn_front_r(100, -30);
    set_brake_fc(10, 30);
    set_brake_bc(10, 30);
    set_delay_f(10);
    
  }
void fw_ch_line(int num)
  {
    while(1)
        {
          Motor(-15, -15);
          delay(5);
          Serial.println(read_sensorA(0));
          if(read_sensorA(0) > 300 && read_sensorA(7) > 300)
            {
              break;
            }
        }
    for(int i=0; i< num; i++)
      {
        while(1)
          {
            delay(5);      
            if(read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(7) > md_sensorA(7)-50)
              {
                Motor(-2 ,10);
              }
            else if(read_sensorA(0) > md_sensorA(0)-50 && read_sensorA(7) < md_sensorA(7)-50)
              {
                Motor(10 ,-2);
              }
            else if(read_sensorA(0) > md_sensorA(0)-50 && read_sensorA(7) > md_sensorA(7)-50)
              {          
                Motor(10 ,10);
              }
            else 
              {
                Motor(-1 ,-1);
                break;
              }      
          }
        if(num > 1)
          {
            
            while(1)
              {
                if(read_sensorA(0) > md_sensorA(0)-50 && read_sensorA(7) > md_sensorA(7)-50)
                  {
                    break;
                  }
                else
                  {
                    Motor(-5 ,-5);
                  }
                delay(5);
              }
            Motor(1 ,1);
          }
      }
    while(1)
          {
            delay(5);      
            if(read_sensorA(0) < md_sensorA(0)-50 && read_sensorA(7) > md_sensorA(7)-50)
              {
                Motor(-2 ,10);
              }
            else if(read_sensorA(0) > md_sensorA(0)-50 && read_sensorA(7) < md_sensorA(7)-50)
              {
                Motor(10 ,-2);
              }
            else if(read_sensorA(0) > md_sensorA(0)-50 && read_sensorA(7) > md_sensorA(7)-50)
              {          
                Motor(10 ,10);
              }
            else 
              {
                Motor(-1 ,-1);
                break;
              }      
          }
    
  }
