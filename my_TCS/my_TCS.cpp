#include "my_TCS.h"

my_TCS::my_TCS(uint16_t eepromOffset) {
    _addr = TCS34725_ADDRESS;
    _wire = &Wire;
    _softWire = nullptr;
    _eepromOffset = eepromOffset;
    calibrated = false;
    _lastSaveOk = false;
    c = r = g = b = 0;
    memset(ref_ratio_r, 0, sizeof(ref_ratio_r));
    memset(ref_ratio_g, 0, sizeof(ref_ratio_g));
    memset(ref_ratio_b, 0, sizeof(ref_ratio_b));
    memset(cal_flags, 0, sizeof(cal_flags));
}

bool my_TCS::begin(uint8_t addr, TwoWire *theWire) {
    if (!theWire) return false;
    _addr = addr; _wire = theWire; _softWire = nullptr;
    return beginSelected();
}
bool my_TCS::begin(uint8_t addr, SoftwareWire *theWire) {
    if (!theWire) return false;
    _addr = addr; _wire = nullptr; _softWire = theWire;
    return beginSelected();
}
void my_TCS::busBegin() { if (_softWire) _softWire->begin(); else _wire->begin(); }
void my_TCS::busBeginTransmission(uint8_t addr) { if (_softWire) _softWire->beginTransmission(addr); else _wire->beginTransmission(addr); }
size_t my_TCS::busWrite(uint8_t value) { return _softWire ? _softWire->write(value) : _wire->write(value); }
uint8_t my_TCS::busEndTransmission(bool stop) { return _softWire ? _softWire->endTransmission(stop) : _wire->endTransmission(stop); }
uint8_t my_TCS::busRequestFrom(uint8_t addr, uint8_t count) { return _softWire ? _softWire->requestFrom(addr,count) : _wire->requestFrom(addr,count); }
int my_TCS::busAvailable() { return _softWire ? _softWire->available() : _wire->available(); }
int my_TCS::busRead() { return _softWire ? _softWire->read() : _wire->read(); }
bool my_TCS::beginSelected() {
    busBegin();
    uint16_t id = 0;
    if (!read16(0x12,id) || ((id & 0xff) != 0x44 && (id & 0xff) != 0x4d)) return false;
    if (!write8(0x00,0x01)) return false;
    delay(3);
    if (!write8(0x00,0x03) || !setConfig()) return false;
    loadColorCalibration();
    return true;
}
bool my_TCS::setConfig(uint8_t it, uint8_t gain) {
    if (!write8(0x01,it) || !write8(0x0f,gain)) return false;
    delay(10); return true;
}
bool my_TCS::readFast() {
    busBeginTransmission(_addr);
    busWrite(0xa0 | 0x13);
    // SoftwareWire 1.6.0 requestFrom starts a new transaction; use STOP.
    if (busEndTransmission(_softWire != nullptr) != 0 ||
        busRequestFrom(_addr,1) != 1 || busAvailable() < 1) return false;
    if (!(busRead() & 1)) return false;
    uint16_t nc,nr,ng,nb;
    if (!read16(0x14,nc) || !read16(0x16,nr) ||
        !read16(0x18,ng) || !read16(0x1a,nb)) return false;
    c=nc; r=nr; g=ng; b=nb; return true;
}

void my_TCS::startColorCalibration() {
    memset(cal_flags,0,sizeof(cal_flags)); calibrated=false; _lastSaveOk=false;
}
bool my_TCS::allColorsCalibrated() const {
    for (uint8_t i=0;i<6;i++) if (!cal_flags[i]) return false;
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
    cr >= ref_ratio_r[3] * 0.90f &&
    cg >= ref_ratio_g[3] * 0.90f &&
    fabs(cr - cg) <= fabs(ref_ratio_r[3] - ref_ratio_g[3]) * 1.3f &&  // ลดจาก 1.8
    cb <= ref_ratio_b[3] * 1.3f && 
    (cr + cg) >= (ref_ratio_r[3] + ref_ratio_g[3]) * 0.90f &&
    cb <= (cr + cg) * 0.40f &&   // ลดจาก 0.50 → B ต้องต่ำชัด
    c > 300 && c < 10000) {      // ปรับ c ให้แคบลง
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

int my_TCS::_Raw() {
    //Serial.printf("  Raw → C:%5u R:%5u G:%5u B:%5u", c, r, g, b);
    if (c > 0) {
        return c;
    } else {
        return -1;
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

bool my_TCS::write8(uint8_t reg, uint8_t val) {
    busBeginTransmission(_addr); busWrite(0x80 | reg); busWrite(val);
    return busEndTransmission() == 0;
}
bool my_TCS::read16(uint8_t reg, uint16_t &value) {
    busBeginTransmission(_addr); busWrite(0xa0 | reg);
    if (busEndTransmission() != 0 || busRequestFrom(_addr,2) != 2 || busAvailable() < 2) return false;
    value = static_cast<uint8_t>(busRead());
    value |= static_cast<uint16_t>(static_cast<uint8_t>(busRead())) << 8;
    return true;
}

// ================== EEPROM functions ==================

bool my_TCS::writeEEPROM(uint16_t eeaddress, uint8_t data) {
    Wire.beginTransmission(EXTERNAL_EEPROM_ADDR);
    Wire.write((uint8_t)(eeaddress >> 8));
    Wire.write((uint8_t)(eeaddress & 0xFF));
    Wire.write(data);
    if (Wire.endTransmission() != 0) return false;
    delay(10);
    Wire.beginTransmission(EXTERNAL_EEPROM_ADDR);
    Wire.write(static_cast<uint8_t>(eeaddress >> 8)); Wire.write(static_cast<uint8_t>(eeaddress));
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(static_cast<uint8_t>(EXTERNAL_EEPROM_ADDR),static_cast<uint8_t>(1)) != 1 || !Wire.available()) return false;
    return static_cast<uint8_t>(Wire.read()) == data;
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

bool my_TCS::writeEEPROM16(uint16_t eeaddress, uint16_t data) {
    return writeEEPROM(eeaddress, data >> 8) && writeEEPROM(eeaddress + 1, data & 0xff);
}

uint16_t my_TCS::readEEPROM16(uint16_t eeaddress) {
    uint16_t high = readEEPROM(eeaddress);
    uint16_t low  = readEEPROM(eeaddress + 1);
    return (high << 8) | low;
}

bool my_TCS::calibrationStorageSafe() {
    const uint32_t first = static_cast<uint32_t>(COLOR_CAL_START_ADDR) + _eepromOffset;
    const uint32_t last = first + 36u;
    if (last > 0x3ffeu ||
        (first != 0x80u && first <= 0xa4u && last >= 0x80u) ||
        (first != 0xc0u && first <= 0xe4u && last >= 0xc0u) ||
        (first != 0x100u && first <= 0x124u && last >= 0x100u) ||
        (first <= 0x295u && last >= 0x200u) ||
        (first <= 0x458u && last >= 0x44cu) ||
        (first <= 0x488u && last >= 0x470u)) return false;
    if (first > 0x95u) return true;
    uint8_t raw[150];
    for (uint16_t i=0; i<150; ++i) {
        Wire.beginTransmission(EXTERNAL_EEPROM_ADDR);
        Wire.write(static_cast<uint8_t>(i >> 8)); Wire.write(static_cast<uint8_t>(i));
        if (Wire.endTransmission(false) != 0 || Wire.requestFrom(static_cast<uint8_t>(EXTERNAL_EEPROM_ADDR),static_cast<uint8_t>(1)) != 1 || !Wire.available()) return false;
        raw[i]=static_cast<uint8_t>(Wire.read());
    }
    uint16_t crc=0xffff;
    for (uint16_t i=0;i<148;++i) {
        crc ^= static_cast<uint16_t>(raw[i]) << 8;
        for (uint8_t bit=0;bit<8;++bit) crc=(crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u) : static_cast<uint16_t>(crc << 1);
    }
    const bool valid = raw[0]==0x31 && raw[1]==0x50 && raw[2]==0x4d && raw[3]==0x4d &&
        raw[4]==1 && raw[5]==0 && raw[6]==150 && raw[7]==0 &&
        (raw[8] & 0xf8u)==0 && raw[9]==0 && raw[10]==0 && raw[11]==0 &&
        crc==static_cast<uint16_t>(raw[148] | (static_cast<uint16_t>(raw[149]) << 8));
    return !valid;
}

void my_TCS::saveColorCalibration() {
    _lastSaveOk = false;
    if (!calibrationStorageSafe()) {
        Serial.println("Color calibration NOT saved: EEPROM conflict or read failed");
        return;
    }
    uint16_t addr = COLOR_CAL_START_ADDR + _eepromOffset;

    uint8_t flags = 0;
    for (int i = 0; i < 6; i++) if (cal_flags[i]) flags |= (1 << i);
    if (!writeEEPROM(addr++, flags)) { Serial.println("EEPROM write failed"); return; }

    for (int i = 0; i < 6; i++) {
        if (!writeEEPROM16(addr, (uint16_t)(ref_ratio_r[i] * 10000))) { Serial.println("EEPROM write failed"); return; } addr += 2;
        if (!writeEEPROM16(addr, (uint16_t)(ref_ratio_g[i] * 10000))) { Serial.println("EEPROM write failed"); return; } addr += 2;
        if (!writeEEPROM16(addr, (uint16_t)(ref_ratio_b[i] * 10000))) { Serial.println("EEPROM write failed"); return; } addr += 2;
    }
    _lastSaveOk = true;
    Serial.print("Saved to EEPROM offset 0x"); Serial.println(_eepromOffset, HEX);
}

void my_TCS::loadColorCalibration() {
    uint16_t addr = COLOR_CAL_START_ADDR + _eepromOffset;

    uint8_t flags = readEEPROM(addr++);
    if (flags & 0xC0) flags = 0; // Unused bits set: erased or invalid EEPROM data
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
