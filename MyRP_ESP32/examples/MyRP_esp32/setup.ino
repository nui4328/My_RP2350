void set_motor()
  {
    
    robot.set_Freq("Coreless_Motors");      // กำหนดความถี่ให้กับมอเตอร์เกาหลีหรือ มอเตอร์ Coreless ("Coreless_Motors")
      //set_Freq("DC_Motors");        // กำหนดความถี่ให้กับมอเตอร์ธรรมกา(DC_Motors)
    
    robot.distance_scale_fw(1.00);       // ปรับค่าเพื่อให้ได้ระยะทางที่เป็นจริงสำหรับเดินหน้า
    robot.distance_scale_bw(1.00);        // ปรับค่าเพื่อให้ได้ระยะทางที่เป็นจริงสำหรับถอยหลัง
   
    robot.set_slow_motor(20, 20);     
    robot.set_turn_center_l(-90, 90);
    robot.set_turn_center_r(90, -90);
    robot.set_turn_front_l(-10, 100);
    robot.set_turn_front_r(100, -10);
    robot.set_brake_fc(10, 30);
    robot.set_brake_bc(10, 30);
    robot.set_delay_f(10);
    
  }

  void _print()
    {
          /*
        for(int i=0; i<8; i++)
          {
            Serial.print(read_sensorA(i));
            Serial.print(" ");
            delay(10);
          }
        Serial.println(" ");
        */
        //Serial.print(encoder.Poss_L());Serial.print("  "); Serial.println(encoder.Poss_R());

        //Serial.print("adc :");Serial.print(ADC_i2c()); Serial.println( "   " );  
      
        //Serial.print("my.gyro('z') :");Serial.print(my.gyro('z')); Serial.print( "   " );  
        //Serial.print( analogRead(7) );   Serial.print( "  " );   Serial.print( analogRead(8) ); Serial.println( "     " );
        //Serial.print(  md_sensorC(0));   Serial.print( "  " );   Serial.println(  md_sensorC(1) ); 

        // Serial.print( sensorMinC[0] );   Serial.print( "   " );   Serial.print( sensorMinC[1] ); 
        //Serial.print( "   " ); 
        // Serial.print( sensorMaxC[0] );   Serial.print( "   " );   Serial.println( sensorMaxC[1] );  
        
        //delay(10);
        //Serial.print(sensorMin_A[0] );
    }