#include "my_TCS.h"

my_TCS::my_TCS(uint16_t eepromOffset) {
    _addr = TCS34725_ADDRESS;
    _wire = &Wire;
    _eepromOffset = eepromOffset;
    calibrated = false;
    memset(cal_flags, 0, sizeof(cal_flags));
}

bool my_TCS::begin(uint8_t addr, TwoWire *theWire) {
    _addr = addr;
    _wire = theWire;

    _wire->begin();

    uint8_t id = read16(0x12) & 0xFF;
    if (id != 0x44 && id != 0x4D) return false;

    write8(0x00, 0x01); delay(3);
    write8(0x00, 0x03);

    setConfig();

    loadColorCalibration();
    return true;
}

bool my_TCS::setConfig(uint8_t it, uint8_t gain) {
    write8(0x01, it);
    write8(0x0F, gain);
    delay(10);
    return true;
}

bool my_TCS::readFast() {
    _wire->beginTransmission(_addr);
    _wire->write(0xA0 | 0x13);
    _wire->endTransmission(false);

    _wire->requestFrom(_addr, (uint8_t)1);
    if (_wire->available() < 1) return false;
    if (!(_wire->read() & 0x01)) return false;

    c = read16(0x14);
    r = read16(0x16);
    g = read16(0x18);
    b = read16(0x1A);
    return true;
}

void my_TCS::calibrateRed()    { _calibrateSingle(0); }
void my_TCS::calibrateGreen()  { _calibrateSingle(1); }
void my_TCS::calibrateBlue()   { _calibrateSingle(2); }
void my_TCS::calibrateYellow() { _calibrateSingle(3); }
void my_TCS::calibrateWhite()  { _calibrateSingle(4); }
void my_TCS::calibrateBlack()  { _calibrateSingle(5); }

void my_TCS::_calibrateSingle(uint8_t idx) {
    float sum_r = 0, sum_g = 0, sum_b = 0;
    int valid = 0;
    const int samples = 8;

    for (int i = 0; i < samples; i++) {
        if (readFast()) {
            delay(30);
            if (c > 10) {
                sum_r += (float)r / c;
                sum_g += (float)g / c;
                sum_b += (float)b / c;
                valid++;
            }
        }
    }

    if (valid > 0) {
        ref_ratio_r[idx] = sum_r / valid;
        ref_ratio_g[idx] = sum_g / valid;
        ref_ratio_b[idx] = sum_b / valid;
        cal_flags[idx] = true;
    }
}

void my_TCS::calibrateDone() {
    calibrated = false;
    for (int i = 0; i < 6; i++) {
        if (cal_flags[i]) {
            calibrated = true;
            break;
        }
    }
    if (calibrated) saveColorCalibration();
}

String my_TCS::getColor() {
    if (!calibrated || c < 30) return "UNKNOWN";

    float cr = (float)r / c;
    float cg = (float)g / c;
    float cb = (float)b / c;

    // 1. BLACK ก่อนเลย - ใช้ clear channel เป็นหลัก (ปรับ threshold ตามสภาพจริงของคุณ)
    if (cal_flags[5] && c < 180) {          // เพิ่มจาก 120 เป็น 180 ถ้าดำของคุณยังสะท้อนแสงนิดหน่อย
        return "BLACK";
    }

    // 2. WHITE - ต้องสว่างมาก + ratio ใกล้เคียงกันมาก (ปรับ ±0.06-0.08 ตามการทดสอบ)
    if (cal_flags[4] && c > 1800 && 
        fabs(cr - ref_ratio_r[4]) < 0.07 &&
        fabs(cg - ref_ratio_g[4]) < 0.07 &&
        fabs(cb - ref_ratio_b[4]) < 0.07) {
        return "WHITE";
    }

    // 3. RED - ใช้ค่าจาก calibration เป็นฐาน
if (cal_flags[0] &&
    cr >= ref_ratio_r[0] * 0.90f &&                  // ต่ำกว่าค่าปกติ 10%
    cr >= cg + (ref_ratio_r[0] - ref_ratio_g[0]) * 0.8f &&  // diff R-G ตาม cal จริง
    cr >= cb + (ref_ratio_r[0] - ref_ratio_b[0]) * 0.8f &&
    c > 250) {
    return "RED";
}

// 4. YELLOW - adaptive เต็มรูปแบบ
if (cal_flags[3] &&
    cr >= ref_ratio_r[3] * 0.88f &&
    cg >= ref_ratio_g[3] * 0.88f &&
    fabs(cr - cg) <= fabs(ref_ratio_r[3] - ref_ratio_g[3]) * 1.8f &&  // diff ตาม cal จริง
    cb <= ref_ratio_b[3] * 1.5f &&                  // ผ่อน cb ตาม cal
    (cr + cg) >= (ref_ratio_r[3] + ref_ratio_g[3]) * 0.88f &&
    cb <= (cr + cg) * 0.50f &&                      // relative ratio
    c > 250 && c < 12000) {
    return "YELLOW";
}

// 5. GREEN - adaptive
if (cal_flags[1] &&
    cg >= ref_ratio_g[1] * 0.90f &&
    cg >= cr + (ref_ratio_g[1] - ref_ratio_r[1]) * 0.8f &&
    cg >= cb + (ref_ratio_g[1] - ref_ratio_b[1]) * 0.8f &&
    c > 250) {
    return "GREEN";
}

// 6. BLUE - adaptive
if (cal_flags[2] &&
    cb >= ref_ratio_b[2] * 0.90f &&
    cb >= cg + (ref_ratio_b[2] - ref_ratio_g[2]) * 0.8f &&
    cb >= cr + (ref_ratio_b[2] - ref_ratio_r[2]) * 0.8f &&
    c > 250) {
    return "BLUE";
}

   // 6. Distance fallback (ปรับให้ YELLOW ทนทานขึ้น ไม่เปลี่ยนเป็น UNKNOWN เร็ว)
float minDist = 1e9;
int best = -1;
const String names[6] = {"RED", "GREEN", "BLUE", "YELLOW", "WHITE", "BLACK"};

for (int i = 0; i < 6; i++) {
    if (!cal_flags[i]) continue;
    float dr = ref_ratio_r[i] - cr;
    float dg = ref_ratio_g[i] - cg;
    float db = ref_ratio_b[i] - cb;
    float dist = dr*dr + dg*dg + db*db;

    // ให้ YELLOW มีน้ำหนักพิเศษ (เพราะ diff ต่ำมาก → ทำให้ชนะง่ายขึ้น)
    if (i == 3) {  // index 3 = YELLOW
        dist *= 0.55f;  // ลดระยะห่างลง 35% → YELLOW ชนะแม้เพี้ยนเล็กน้อย
    }

    if (dist < minDist) {
        minDist = dist;
        best = i;
    }
}

// ปรับ threshold ให้กว้างขึ้นเล็กน้อยสำหรับความเป็นจริง (0.018 → 0.025-0.030)
if (best == -1 || minDist > 0.035f) {  // ลอง 0.025f ก่อน ถ้ายังเปลี่ยนเร็วค่อยลดเหลือ 0.022f
    // Debug print แบบละเอียด
    Serial.print("UNKNOWN (fallback) - cr:"); Serial.print(cr,3);
    Serial.print(" cg:"); Serial.print(cg,3);
    Serial.print(" cb:"); Serial.print(cb,3);
    Serial.print(" c:"); Serial.print(c);
    Serial.print(" minDist:"); Serial.print(minDist,4);
    if (best != -1) {
        Serial.print(" closest: "); Serial.print(names[best]);
        Serial.print(" dist_to_best: "); Serial.println(minDist,4);
    } else {
        Serial.println(" no close match at all");
    }
    return "UNKNOWN";
}

// Safeguard สำหรับ BLACK: ถ้าสว่างเกิน → ไม่น่าใช่ BLACK
if (best == 5 && c > 450) {  // เพิ่มเป็น 450 เพื่อให้ BLACK ต้องมืดจริง ๆ
    Serial.print("BLACK rejected by brightness - c="); Serial.println(c);
    return "UNKNOWN";
}

return names[best];

    //---------------------
}

void my_TCS::printRaw() {
    Serial.printf("  Raw → C:%5u R:%5u G:%5u B:%5u", c, r, g, b);
    if (c > 0) {
        Serial.printf("  ratio R:%.3f G:%.3f B:%.3f\n", (float)r/c, (float)g/c, (float)b/c);
    } else {
        Serial.println("  (มืด/ไม่มีแสง)");
    }
}

void my_TCS::printCalibration() {
    if (!calibrated) {
        Serial.println("  ยังไม่เคย calibrate");
        return;
    }
    const char* names[] = {"RED","GREEN","BLUE","YELLOW","WHITE","BLACK"};
    Serial.println("  ค่า calibration (ratio):");
    for (int i = 0; i < 6; i++) {
        if (cal_flags[i]) {
            Serial.printf("  %s → R:%.3f  G:%.3f  B:%.3f\n",
                          names[i], ref_ratio_r[i], ref_ratio_g[i], ref_ratio_b[i]);
        }
    }
}

// ================== I2C functions ==================

void my_TCS::write8(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(0x80 | reg);
    _wire->write(val);
    _wire->endTransmission();
}

uint16_t my_TCS::read16(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(0xA0 | reg);
    _wire->endTransmission();
    _wire->requestFrom(_addr, (uint8_t)2);
    if (_wire->available() < 2) return 0;
    uint16_t val = _wire->read();
    val |= ((uint16_t)_wire->read()) << 8;
    return val;
}

// ================== EEPROM functions ==================

void my_TCS::writeEEPROM(uint16_t eeaddress, uint8_t data) {
    Wire.beginTransmission(EXTERNAL_EEPROM_ADDR);
    Wire.write((uint8_t)(eeaddress >> 8));
    Wire.write((uint8_t)(eeaddress & 0xFF));
    Wire.write(data);
    Wire.endTransmission();
    delay(10);
}

uint8_t my_TCS::readEEPROM(uint16_t eeaddress) {
    Wire.beginTransmission(EXTERNAL_EEPROM_ADDR);
    Wire.write((uint8_t)(eeaddress >> 8));
    Wire.write((uint8_t)(eeaddress & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom(EXTERNAL_EEPROM_ADDR, (uint8_t)1);
    if (Wire.available()) return Wire.read();
    return 0;
}

void my_TCS::writeEEPROM16(uint16_t eeaddress, uint16_t data) {
    writeEEPROM(eeaddress, data >> 8);
    writeEEPROM(eeaddress + 1, data & 0xFF);
}

uint16_t my_TCS::readEEPROM16(uint16_t eeaddress) {
    uint16_t high = readEEPROM(eeaddress);
    uint16_t low  = readEEPROM(eeaddress + 1);
    return (high << 8) | low;
}

void my_TCS::saveColorCalibration() {
    uint16_t addr = COLOR_CAL_START_ADDR + _eepromOffset;

    uint8_t flags = 0;
    for (int i = 0; i < 6; i++) if (cal_flags[i]) flags |= (1 << i);
    writeEEPROM(addr++, flags);

    for (int i = 0; i < 6; i++) {
        writeEEPROM16(addr, (uint16_t)(ref_ratio_r[i] * 10000)); addr += 2;
        writeEEPROM16(addr, (uint16_t)(ref_ratio_g[i] * 10000)); addr += 2;
        writeEEPROM16(addr, (uint16_t)(ref_ratio_b[i] * 10000)); addr += 2;
    }
    Serial.print("Saved to EEPROM offset 0x"); Serial.println(_eepromOffset, HEX);
}

void my_TCS::loadColorCalibration() {
    uint16_t addr = COLOR_CAL_START_ADDR + _eepromOffset;

    uint8_t flags = readEEPROM(addr++);
    for (int i = 0; i < 6; i++) cal_flags[i] = flags & (1 << i);

    for (int i = 0; i < 6; i++) {
        ref_ratio_r[i] = (float)readEEPROM16(addr) / 10000.0; addr += 2;
        ref_ratio_g[i] = (float)readEEPROM16(addr) / 10000.0; addr += 2;
        ref_ratio_b[i] = (float)readEEPROM16(addr) / 10000.0; addr += 2;
    }

    calibrated = false;
    for (int i = 0; i < 6; i++) if (cal_flags[i]) { calibrated = true; break; }

    Serial.print("Loaded from EEPROM offset 0x"); Serial.println(_eepromOffset, HEX);
    if (calibrated) Serial.println("  → OK");
    else Serial.println("  → ไม่พบ calibration");
}