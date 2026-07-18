void s2()
  {
    
   go_mission_on_floor();  // เข้าคีบที่พื้น

   go_mission_on_stand(); //เข้าคีบบนแท่น 5 เซนติเมตร
  
  ///---------------------------------------------------------------------------เอากระป๋องออกมาจากที่เก็บ วางบนแทน 10 cm
   go_to_mission_on_stand('L', arm_shoulderL_begin);     // arm_shoulderL_begin คือค่า ชี้ไปข้างหน้า
   go_to_mission_on_stand('L', arm_shoulderL_begin + 20);     // วางกระป๋อง ด้านขอบแท่น 
   go_to_mission_on_stand('L', arm_shoulderL_begin - 30);     // วางกระป๋อง กลางแท่น 

   go_to_mission_on_stand('R', arm_shoulderL_begin);          //############ วางบนแท่นปกติ
   go_to_mission_on_stand('R', arm_shoulderL_begin - 30);     //############ วางบนแท่นตรงกลาง
   go_to_mission_on_stand('R', arm_shoulderL_begin + 20);     //############ วางบนแท่นตรงด้านขอบ

   


  }