// =====================================================================
//  ColorSensor.h  -  Minimal non-blocking TCS34725 driver + colour classifier
//
//  Identical copy lives in RGBSensorTest/ and DeliveryRobot/ so the
//  calibration values you measure in the test project work unchanged
//  on the robot. If you change one, copy it to the other.
//
//  Wiring (own I2C bus "Wire1", separate from the OLED on GPIO 11/12):
//    TCS34725 VIN -> 3V3        GND -> GND
//    TCS34725 SDA -> GPIO 1     SCL -> GPIO 2
//    TCS34725 LED -> GPIO 14    (optional, lets the code switch the white LED)
//    TCS34725 INT -> not connected
//  If the sensor isn't found, try swapping the two pin numbers below.
// =====================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>

// ---------------- Sensor settings (tune these) ----------------
#define TCS_ADDR        0x29
#define TCS_SDA_PIN     1
#define TCS_SCL_PIN     2
#define TCS_I2C_FREQ    400000
#define TCS_WIRE        Wire1   // second I2C controller (OLED uses Wire)
#define TCS_LED_PIN     14      // set to -1 if LED pin is not wired

// Integration time: 0xF6 = 24 ms (fast, good while driving)
//                   0xEB = 50 ms, 0xD5 = 101 ms (more sensitive)
#define TCS_ATIME       0xEB    // 50 ms - calibrated setting (sensor 2 cm above track)
// Gain: 0 = 1x, 1 = 4x, 2 = 16x, 3 = 60x
#define TCS_GAIN        2       // 16x  - calibrated setting (sensor 2 cm above track)
// NOTE: COLOR_REFS below were measured at 4x / 24 ms at ~5-10 mm.
// Percentages barely change with gain/time, but DO change with distance:
// re-measure all colours at the new mounting height.

// ---------------- Colour classes ----------------
enum CardColor { COLOR_NONE = 0, COLOR_RED, COLOR_GREEN, COLOR_BLUE };

inline const char* colorName(CardColor c) {
  switch (c) {
    case COLOR_RED:   return "RED";
    case COLOR_GREEN: return "GREEN";
    case COLOR_BLUE:  return "BLUE";
    default:          return "NONE";
  }
}

// ---------------- Calibration (FILL THESE IN!) ----------------
// Run RGBSensorTest, hold the sensor over each surface at the height it
// will be mounted on the robot, and copy the "r g b" percentages shown
// on the OLED (r/c, g/c, b/c x 100) into the table below.
// "clear" is the raw C value - used to reject very dark surfaces.
struct ColorRef { CardColor color; float r, g, b; const char* label; };

static const ColorRef COLOR_REFS[] = {
  //  class         r%    g%    b%    label
  { COLOR_RED,     71.7, 16.1, 12.6, "RED"   },  // MEASURED @2cm 16x/50ms (C~1230), 26/26 reads
  { COLOR_GREEN,   39.1, 35.6, 19.2, "GREEN" },  // MEASURED @2cm 16x/50ms (C~2155) - only ~4.6 from WHITE!
  { COLOR_BLUE,    36.7, 35.2, 22.5, "BLUE"  },  // MEASURED @2cm 16x/50ms (C~2205) - only ~4.1 from GREEN!
  { COLOR_NONE,    42.0, 32.1, 20.1, "WHITE" },  // floor - MEASURED @2cm 16x/50ms (C~4000)
  { COLOR_NONE,    34.0, 34.0, 30.0, "BLACK" },  // line (also caught by MIN_CLEAR)
};
static const int NUM_COLOR_REFS = sizeof(COLOR_REFS) / sizeof(COLOR_REFS[0]);

// Readings with a clear value below this are treated as "too dark" (black line / nothing)
#define COLOR_MIN_CLEAR     150
// If the best match is further than this (in % units), treat as unknown
#define COLOR_MAX_DISTANCE  12.0f

// ---------------- Data ----------------
struct ColorReading {
  uint16_t c, r, g, b;      // raw counts
  float rp, gp, bp;         // r/c, g/c, b/c in percent
  int   refIndex;           // index into COLOR_REFS of best match, -1 = none
  float distance;           // distance to that reference
  CardColor color;          // final classification
};

// ---------------- Low level I2C ----------------
inline void tcsWrite8(uint8_t reg, uint8_t val) {
  TCS_WIRE.beginTransmission(TCS_ADDR);
  TCS_WIRE.write(0x80 | reg);           // command bit
  TCS_WIRE.write(val);
  TCS_WIRE.endTransmission();
}

inline uint8_t tcsRead8(uint8_t reg) {
  TCS_WIRE.beginTransmission(TCS_ADDR);
  TCS_WIRE.write(0x80 | reg);
  if (TCS_WIRE.endTransmission(false) != 0) return 0xFF;
  if (TCS_WIRE.requestFrom((uint8_t)TCS_ADDR, (uint8_t)1) != 1) return 0xFF;
  return TCS_WIRE.read();
}

inline void colorSensorLed(bool on) {
  if (TCS_LED_PIN >= 0) digitalWrite(TCS_LED_PIN, on ? HIGH : LOW);
}

// Returns the chip ID (0x44 = TCS34725, 0x4D = TCS34727), or 0 if not found.
inline uint8_t colorSensorBegin() {
  static bool busStarted = false;
  if (!busStarted) {
    TCS_WIRE.begin(TCS_SDA_PIN, TCS_SCL_PIN, TCS_I2C_FREQ);
    busStarted = true;
  }
  if (TCS_LED_PIN >= 0) { pinMode(TCS_LED_PIN, OUTPUT); colorSensorLed(true); }

  uint8_t id = tcsRead8(0x12);
  if (id != 0x44 && id != 0x4D && id != 0x10) return 0;   // 0x10 = some clones

  tcsWrite8(0x01, TCS_ATIME);       // ATIME
  tcsWrite8(0x0F, TCS_GAIN);        // CONTROL (gain)
  tcsWrite8(0x00, 0x01);            // ENABLE: PON
  delay(3);
  tcsWrite8(0x00, 0x03);            // ENABLE: PON | AEN
  return id;
}

// Classify a reading by nearest calibrated reference.
inline void classifyColor(ColorReading& rd) {
  rd.refIndex = -1;
  rd.distance = 999.0f;
  rd.color = COLOR_NONE;
  if (rd.c == 0) { rd.rp = rd.gp = rd.bp = 0; return; }

  rd.rp = 100.0f * rd.r / rd.c;
  rd.gp = 100.0f * rd.g / rd.c;
  rd.bp = 100.0f * rd.b / rd.c;

  if (rd.c < COLOR_MIN_CLEAR) return;   // too dark

  for (int i = 0; i < NUM_COLOR_REFS; i++) {
    float dr = rd.rp - COLOR_REFS[i].r;
    float dg = rd.gp - COLOR_REFS[i].g;
    float db = rd.bp - COLOR_REFS[i].b;
    float d = sqrtf(dr * dr + dg * dg + db * db);
    if (d < rd.distance) { rd.distance = d; rd.refIndex = i; }
  }
  if (rd.refIndex >= 0 && rd.distance <= COLOR_MAX_DISTANCE) {
    rd.color = COLOR_REFS[rd.refIndex].color;
  }
}

// Non-blocking: returns true and fills `out` only when a new conversion is
// ready (about every 24 ms with the default ATIME). Never waits.
inline bool colorSensorRead(ColorReading& out) {
  // AVALID stays set once the first cycle finishes, so pace reads by the
  // integration time to avoid returning the same sample twice.
  static uint32_t lastReadMs = 0;
  const uint32_t periodMs = (uint32_t)((256 - TCS_ATIME) * 2.4f) + 1;
  if (millis() - lastReadMs < periodMs) return false;

  uint8_t status = tcsRead8(0x13);
  if (status == 0xFF || !(status & 0x01)) return false;   // AVALID not set yet

  TCS_WIRE.beginTransmission(TCS_ADDR);
  TCS_WIRE.write(0xA0 | 0x14);          // command + auto-increment, start at CDATAL
  if (TCS_WIRE.endTransmission(false) != 0) return false;
  if (TCS_WIRE.requestFrom((uint8_t)TCS_ADDR, (uint8_t)8) != 8) return false;

  lastReadMs = millis();
  uint8_t buf[8];
  for (int i = 0; i < 8; i++) buf[i] = TCS_WIRE.read();
  out.c = buf[0] | (buf[1] << 8);
  out.r = buf[2] | (buf[3] << 8);
  out.g = buf[4] | (buf[5] << 8);
  out.b = buf[6] | (buf[7] << 8);
  classifyColor(out);
  return true;
}
