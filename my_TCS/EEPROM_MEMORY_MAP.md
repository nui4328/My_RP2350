# EEPROM memory map — my_TCS, MyMINI_16Pro, BNO055

ตรวจจาก source ในเครื่อง วันที่ 31 ตุลาคม 2026 เป็นแผนที่จากโค้ด ไม่ได้อ่าน EEPROM จริงหรือยืนยันชนิดอุปกรณ์บนบอร์ด

ไลบรารีทั้งสามอ้างถึง I2C address 0x50; my_TCS ใช้ Wire (GP4/GP5) สำหรับ EEPROM แม้ตัวเซนเซอร์สีจะต่อบัสอื่น และ BNO055 ใช้บัสที่กำหนดให้ตัวไจโร (MyMINI_16Pro เลือก Wire สำหรับ BNO055) หากบัสหรือชิปจริงต่างกัน ตารางนี้ไม่ใช่ EEPROM ก้อนเดียวกัน

| ข้อมูล | Start | End รวมปลาย | ขนาด | สถานะ |
|---|---:|---:|---:|---|
| MyMINI_16Pro legacy | 0x0000 | 0x0095 | 150 | fallback อ่านและย้ายเมื่อใช้ได้; ทับ TCS0 22 ไบต์ |
| TCS0 offset 0x0000 | 0x0080 | 0x00A4 | 37 | ทับ legacy 0x0080–0x0095 |
| TCS1 offset 0x0040 | 0x00C0 | 0x00E4 | 37 | ไม่ชนรายการอื่นในตาราง |
| TCS2 offset 0x0080 | 0x0100 | 0x0124 | 37 | ใช้โดยตัวอย่างใหม่ |
| MyMINI_16Pro ปัจจุบัน | 0x0200 | 0x0295 | 150 | บันทึกและตรวจสอบ |
| BNO055 gyro offset | 0x044C | 0x0458 | 13 | บันทึกด้วย calibrate |
| BNO055 fusion profile | 0x0470 | 0x0488 | 25 | โหลดตอน begin; บันทึกด้วยคำสั่งบันทึก profile |
| EEPROM diagnostic | 0x3FFF | 0x3FFF | 1 | ทดลองเขียนและคืนข้อมูลเดิมเมื่อสั่ง diagnostic |

`CalibrationData` เป็น packed struct มี static_assert 150 bytes: header 12 bytes, front และ rear min/max รวม 128 bytes, center min/max 8 bytes และ CRC 2 bytes. สีหนึ่ง instance บันทึก flags 1 byte + 6 สี × 3 ratio × 2 bytes = 37 bytes. ฟังก์ชัน `calibrateDone()` ของ my_TCS เรียกบันทึกเมื่อมี flags อย่างน้อยหนึ่งสี; `begin()` โหลดค่า แต่ไม่เขียนค่าสี. `loadCalibration()` ของ MyMINI โหลด 0x0200 ก่อน ถ้าอ่านได้แต่ข้อมูลไม่ผ่าน จะลอง 0x0000 และถ้า legacy ถูกต้องจะบันทึกคัดลอกไป 0x0200 อัตโนมัติ. ไม่ลบ legacy.

พื้นที่ที่ไม่พบการจัดสรรในสามไลบรารีที่ตรวจ (รวมปลาย): 0x00A5–0x00BF, 0x00E5–0x00FF, 0x0125–0x01FF, 0x0296–0x044B, 0x0459–0x046F และ 0x0489–0x3FFE. ไม่ได้พิสูจน์ว่าพื้นที่เหล่านี้ว่างบนอุปกรณ์จริงหรือไม่มีการใช้จากไลบรารีอื่น. 0x3FFF สงวน diagnostic.

MyMINI source เรียกอุปกรณ์ CAT24C128 และกำหนดช่วงสูงสุด 0x3FFF (16,384 bytes รวม diagnostic) แต่ยังไม่ได้ตรวจชนิดหรือความจุ EEPROM บนฮาร์ดแวร์จริง. my_TCS ฉบับที่ปรับเพิ่มการตรวจช่วงคาลิเบรตที่สงวนไว้ ปฏิเสธ offset ที่ซ้อนช่วงสีตัวอื่นและที่เก็บคาลิเบรตหลัก; หากบันทึกทับช่วง legacy จะตรวจ CRC และปฏิเสธเมื่อ legacy ถูกต้องหรืออ่าน EEPROM ไม่สำเร็จ. Legacy ที่เสียหายไม่สามารถยืนยันหรือปกป้องได้ด้วยการตรวจ CRC นี้. ต้องสำรอง/ตรวจข้อมูลจริงก่อนตัดสินใจเรื่อง legacy ห้ามใช้ตารางนี้เป็นหลักฐานว่าข้อมูลว่างหรือปลอดภัยต่อการเขียน.

การใช้ขา GP26/GP27 เป็น Wire1 ซ้อนกับสัญญาณ ADC MUX หลัง/หน้าของ MyMINI_16Pro ตามลำดับ; GP28 ใช้ servo ยกในตัวอย่าง MyMINI_16Pro_color. ตัวอย่าง TCS แบบ standalone กับการใช้งานรวมกับหุ่นยนต์จึงมีข้อจำกัดต่างกัน ต้องยืนยันการต่อและจัดสรรขาก่อน.

แหล่งในเครื่อง: my_TCS/my_TCS.h และ my_TCS.cpp; MyMINI_16Pro/src/CalibrationManager.h, CalibrationManager.cpp, I2CDevices.cpp, RobotConfig.h; BNO055/BNO055.h, BNO055.cpp. ติดตั้ง SoftwareWire 1.6.0 และปรับสำเนาให้ใช้ GPIO API บน Arduino-Pico; การทำงานจริงรวม timing และแรงดันยังต้องทดสอบบนบอร์ด.
