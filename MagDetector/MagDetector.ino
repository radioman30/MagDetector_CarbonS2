// ============================================================
//  MagDetector - GroundStudio Carbon S2 (ESP32-S2FN4R2)
//  4 moduri, comutate cu butonul BOOT (GPIO0):
//    1. Busola      - HMC5883L, heading cu calibrare hard-iron
//    2. Poli magnet - N/S + intensitate camp (gain redus)
//    3. Cabluri AC  - detectie camp 50 Hz (varianta pe axe)
//    4. Meteo       - BME280: temperatura, umiditate, presiune
//  Apasare lunga (>1.5s) in modul Busola = calibrare (roteste in 8)
//
//  Hardware:
//    I2C  SDA=GPIO8 SCL=GPIO9  -> HMC5883L (0x1E) + BME280 (0x77)
//    TFT ST7735 1.8" 128x160   -> SCK=36 MOSI=35 CS=37 DC=38 RST=33 BLK=14
//    Buton = BOOT (GPIO0), Buzzer optional = GPIO5
// ============================================================

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>

// ---------------- Pini ----------------
#define PIN_SDA       8
#define PIN_SCL       9
#define PIN_TFT_SCK   36
#define PIN_TFT_MOSI  35
#define PIN_TFT_CS    37
#define PIN_TFT_DC    38
#define PIN_TFT_RST   33
#define PIN_TFT_BLK   14
#define PIN_BTN       0     // butonul BOOT de pe placa
#define PIN_BTN_EXT   4     // buton extern la GND (pull-up intern)
#define PIN_BUZZER    5     // buzzer pasiv la GND
// GPIO3 e nod triplu pe Carbon S2 (confirmat din PCB REV0.0.2):
//  - divizor 442k/160k spre +BAT (citire tensiune)
//  - pinul CHRG/STAT al charger-ului BRCL4054: tras la ~0V cat timp incarca
//  - LED-ul verde de charging (alimentat din USB_VCC prin 20k, jumper J2)
// => tensiunea e valida DOAR pe baterie (fara USB); pe USB afisam doar starea
#define PIN_BAT_ADC   3
#define PIN_VBUS      21      // HIGH = alimentare 5V pe USB prezenta
#define BAT_DIV       3.790f  // teoretic 3.7625 (602k/160k), calibrat cu multimetrul: 3.445V real / 3.42V afisat

// Varianta de tab a display-ului: daca imaginea e deplasata sau culorile
// sunt ciudate, incearca INITR_GREENTAB sau INITR_REDTAB
#define TAB_VARIANT   INITR_BLACKTAB

// Declinatia magnetica pentru Romania (~6.3 grade Est, 2026)
#define DECLINATIE_DEG 6.3f

// Inclinatia magnetica pentru Romania (~63 grade - campul "intra" in pamant).
// Referinta pentru bula de nivel; ajusteaza +/- cateva grade daca bula
// nu sta centrata cand aparatul e perfect orizontal (offset de montaj senzor).
#define DIP_REF_DEG 63.0f

// ---------------- HMC5883L ----------------
#define HMC_ADDR      0x1E
#define HMC_REG_CRA   0x00
#define HMC_REG_CRB   0x01
#define HMC_REG_MODE  0x02
#define HMC_REG_DATA  0x03
#define HMC_REG_STAT  0x09

// uT per LSB pentru cele doua gain-uri folosite
#define UT_PER_LSB_1_3GA (100.0f / 1090.0f)  // gain 1.3 Ga (busola, cabluri)
#define UT_PER_LSB_8_1GA (100.0f / 230.0f)   // gain 8.1 Ga (magneti)

Adafruit_ST7735 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
Adafruit_BME280 bme;
Preferences prefs;
bool bmeOk = false;

// ---------------- Moduri ----------------
enum Mode { MODE_BUSOLA = 0, MODE_MAGNET, MODE_CABLU, MODE_METEO, NUM_MODES };
Mode mode = MODE_BUSOLA;

// Calibrare hard-iron (offset) + soft-iron simplu (scala)
float offX = 0, offY = 0, offZ = 0;
float sclX = 1, sclY = 1, sclZ = 1;

// Filtru exponential pt busola
float fX = 0, fY = 0, fZ = 0;
bool  fInit = false;
int   lastNeedleAngle = -999;

// Buffer circular pt detectia AC (modul cabluri)
// N=96 la fs=75Hz: 50Hz aliazeaza pe 25Hz = exact bin-ul k=32 al ferestrei
// (si armonica de 100Hz aliazeaza tot pe 25Hz - se cumuleaza, ne ajuta)
#define AC_N 96
float acX[AC_N], acY[AC_N], acZ[AC_N];
int   acIdx = 0, acCount = 0;
#define HIST_N 128
uint8_t hist[HIST_N];
int histIdx = 0;

// Buton
bool     btnWasDown = false;
uint32_t btnDownAt = 0;
bool     longFired = false;

uint32_t tSample = 0, tUi = 0, tBat = 0;
float curUtPerLsb = UT_PER_LSB_1_3GA;

// ---------------- Driver HMC5883L ----------------
void hmcWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// cra: averaging+rata, crb: gain. Porneste modul continuu.
void hmcConfig(uint8_t cra, uint8_t crb) {
  hmcWrite(HMC_REG_CRA, cra);
  hmcWrite(HMC_REG_CRB, crb);
  hmcWrite(HMC_REG_MODE, 0x00);  // continuu
  curUtPerLsb = ((crb >> 5) == 0b111) ? UT_PER_LSB_8_1GA : UT_PER_LSB_1_3GA;
  delay(10);
}

bool hmcDataReady() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(HMC_REG_STAT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(HMC_ADDR, 1) != 1) return false;
  return Wire.read() & 0x01;
}

// Ordinea registrelor HMC5883L este X, Z, Y (big endian)!
bool hmcRead(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(HMC_REG_DATA);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(HMC_ADDR, 6) != 6) return false;
  x = (Wire.read() << 8) | Wire.read();
  z = (Wire.read() << 8) | Wire.read();
  y = (Wire.read() << 8) | Wire.read();
  return true;
}

bool hmcSaturated(int16_t x, int16_t y, int16_t z) {
  return x == -4096 || y == -4096 || z == -4096;
}

// ---------------- UI comun ----------------
const uint16_t modeColor[NUM_MODES] = {ST77XX_BLUE, ST77XX_RED, ST77XX_ORANGE, 0x0560};
const char* modeName[NUM_MODES] = {"BUSOLA", "MAGNET", "CABLURI", "METEO"};

float readBatV() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_BAT_ADC);
  return (mv / 8) * BAT_DIV / 1000.0f;
}

// curba de descarcare LiPo 1S, aproximata pe segmente
int batPercent(float v) {
  const float vp[] = {3.30, 3.50, 3.70, 3.80, 3.95, 4.15};
  const int   pp[] = {0,    10,   25,   50,   75,   100};
  if (v <= vp[0]) return 0;
  if (v >= vp[5]) return 100;
  for (int i = 5; i > 0; i--)
    if (v >= vp[i - 1])
      return pp[i - 1] + (v - vp[i - 1]) * (pp[i] - pp[i - 1]) / (vp[i] - vp[i - 1]);
  return 0;
}

void drawBattery() {
  float v = readBatV();
  tft.fillRect(62, 0, 66, 16, modeColor[mode]);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(66, 4);
  if (digitalRead(PIN_VBUS)) {
    // pe USB: STAT jos = incarcare; sus = plin (citirea e alterata de LED)
    tft.print(v < 1.0f ? "se incarca" : "USB: plin");
  } else {
    tft.printf("%.2fV %d%%", v, batPercent(v));
  }
}

void drawHeader() {
  tft.fillRect(0, 0, 128, 16, modeColor[mode]);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(4, 4);
  tft.print(modeName[mode]);
  drawBattery();
}

void drawFooter(const char* msg) {
  tft.fillRect(0, 150, 128, 10, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(0x7BEF);  // gri
  tft.setCursor(2, 151);
  tft.print(msg);
}

// ---------------- Busola ----------------
// Literele sunt separate ca sa le putem redesena dupa stergerea acului
// (varful acului trece peste ele si le "manca")
void drawCompassLabels() {
  const int cx = 64, cy = 78, R = 50;
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_RED);   tft.setCursor(cx - 2, cy - R + 5);  tft.print("N");
  tft.setTextColor(ST77XX_WHITE); tft.setCursor(cx + R - 12, cy - 3); tft.print("E");
  tft.setCursor(cx - 2, cy + R - 11); tft.print("S");
  tft.setCursor(cx - R + 7, cy - 3);  tft.print("V");
}

void drawCompassFace() {
  const int cx = 64, cy = 78, R = 50;
  tft.drawCircle(cx, cy, R, ST77XX_WHITE);
  tft.drawCircle(cx, cy, R + 1, ST77XX_WHITE);
  for (int a = 0; a < 360; a += 30) {
    float r = radians(a);
    int x1 = cx + (R - 4) * sinf(r), y1 = cy - (R - 4) * cosf(r);
    int x2 = cx + (R - 1) * sinf(r), y2 = cy - (R - 1) * cosf(r);
    tft.drawLine(x1, y1, x2, y2, ST77XX_WHITE);
  }
  drawCompassLabels();
}

// Bula de nivel "magnetica": compara inclinatia masurata a campului terestru
// cu cea de referinta (63 grade). Fara accelerometru detecteaza bine
// inclinarea in planul nord-sud si slab pe cea laterala (est-vest).
void drawBubble(float devDeg) {
  const int bx = 2, by = 19, bw = 34, bh = 11;
  tft.fillRect(bx, by, bw, bh, ST77XX_BLACK);
  tft.drawRoundRect(bx, by, bw, bh, 4, 0x7BEF);
  // reperele centrale ale nivelei
  tft.drawFastVLine(bx + bw / 2 - 4, by + 2, bh - 4, 0x39E7);
  tft.drawFastVLine(bx + bw / 2 + 4, by + 2, bh - 4, 0x39E7);
  bool ok = fabsf(devDeg) < 3.0f;                       // "drept" = sub 3 grade
  int dx = (int)constrain(devDeg * 1.5f, -11.0f, 11.0f); // 1.5 px / grad
  tft.fillCircle(bx + bw / 2 + dx, by + bh / 2, 3, ok ? ST77XX_GREEN : ST77XX_YELLOW);
}

void drawNeedle(int angleDeg, bool erase) {
  const int cx = 64, cy = 78, L = 40;
  float r = radians((float)angleDeg);
  int nx = cx + L * sinf(r), ny = cy - L * cosf(r);
  int tx = cx - (L / 2) * sinf(r), ty = cy + (L / 2) * cosf(r);
  uint16_t cN = erase ? ST77XX_BLACK : ST77XX_RED;
  uint16_t cS = erase ? ST77XX_BLACK : 0x7BEF;
  tft.drawLine(cx, cy, nx, ny, cN);
  tft.drawLine(cx + 1, cy, nx + 1, ny, cN);
  tft.drawLine(cx, cy, tx, ty, cS);
  if (!erase) tft.fillCircle(cx, cy, 3, ST77XX_WHITE);
}

const char* cardinal(float h) {
  static const char* dir[] = {"N", "NE", "E", "SE", "S", "SV", "V", "NV"};
  return dir[((int)((h + 22.5f) / 45.0f)) % 8];
}

int lastBubbleX = -999;

void uiBusola(float mx, float my, float mz) {
  // filtrare vectoriala ca sa nu tremure acul
  if (!fInit) { fX = mx; fY = my; fZ = mz; fInit = true; }
  fX += 0.25f * (mx - fX);
  fY += 0.25f * (my - fY);
  fZ += 0.25f * (mz - fZ);

  float heading = degrees(atan2f(fY, fX)) + DECLINATIE_DEG;
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;

  // acul arata Nordul pe cadran (rotit invers fata de heading)
  int needle = (int)(360 - heading) % 360;
  if (needle != lastNeedleAngle) {
    if (lastNeedleAngle > -999) drawNeedle(lastNeedleAngle, true);
    drawCompassLabels();  // stergerea acului trece peste litere - le refacem
    drawNeedle(needle, false);
    lastNeedleAngle = needle;
  }

  // bula: deviatia inclinatiei magnetice fata de referinta
  float dip = degrees(atan2f(fabsf(fZ), sqrtf(fX * fX + fY * fY)));
  int bubX = (int)constrain((dip - DIP_REF_DEG) * 1.5f, -11.0f, 11.0f);
  if (bubX != lastBubbleX) {  // redesenam doar cand se misca, sa nu clipeasca
    drawBubble(dip - DIP_REF_DEG);
    lastBubbleX = bubX;
  }

  tft.fillRect(0, 132, 128, 16, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(22, 132);
  tft.printf("%3.0f%c %s", heading, 247, cardinal(heading));
}

// ---------------- Poli magnet ----------------
char lastPole = '?';
void uiMagnet(int16_t rx, int16_t ry, int16_t rz, bool sat) {
  float bx = rx * curUtPerLsb, by = ry * curUtPerLsb, bz = rz * curUtPerLsb;
  float mag = sqrtf(bx * bx + by * by + bz * bz);

  tft.fillRect(0, 20, 128, 128, ST77XX_BLACK);
  tft.setTextSize(1);

  if (sat) {
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_MAGENTA);
    tft.setCursor(10, 55);
    tft.print("SATURAT");
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(14, 90);
    tft.print("departeaza magnetul");
    return;
  }

  if (mag < 150) {  // doar campul terestru -> niciun magnet aproape
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(16, 60);
    tft.print("Apropie un magnet");
    tft.setCursor(24, 100);
    tft.printf("camp: %.1f uT", mag);
    lastPole = '?';
    return;
  }

  // polaritatea dupa axa dominanta (Z = fata senzorului)
  // semn inversat: verificat cu busola pe modulul real
  bool north = bz < 0;
  tft.setTextSize(7);
  tft.setTextColor(north ? ST77XX_RED : ST77XX_BLUE);
  tft.setCursor(44, 40);
  tft.print(north ? "N" : "S");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(north ? 34 : 40, 96);
  tft.print(north ? "polul NORD" : "polul SUD");

  // bargraph intensitate (log, 150..2500 uT)
  float frac = constrain((log10f(mag) - log10f(150)) / (log10f(2500) - log10f(150)), 0.0f, 1.0f);
  tft.drawRect(9, 112, 110, 12, ST77XX_WHITE);
  tft.fillRect(11, 114, (int)(106 * frac), 8, north ? ST77XX_RED : ST77XX_BLUE);
  tft.setCursor(34, 130);
  tft.printf("%.0f uT", mag);
}

// ---------------- Cabluri AC ----------------
// Amplitudinea componentei de 25 Hz (= 50 Hz aliazat) prin filtru Goertzel.
// N=96, k=32 -> coeff = 2*cos(2*pi*32/96) = -1, deci recurenta e triviala.
float goertzel50Hz(float* buf, int startIdx) {
  float mean = 0;
  for (int i = 0; i < AC_N; i++) mean += buf[(startIdx + i) % AC_N];
  mean /= AC_N;
  float s1 = 0, s2 = 0;
  for (int i = 0; i < AC_N; i++) {
    float s = (buf[(startIdx + i) % AC_N] - mean) - s1 - s2;
    s2 = s1;
    s1 = s;
  }
  float power = s1 * s1 + s2 * s2 + s1 * s2;
  return 2.0f * sqrtf(max(power, 0.0f)) / AC_N;  // amplitudine varf, uT
}

void uiCablu() {
  if (acCount < AC_N) return;  // asteapta umplerea ferestrei
  float gx = goertzel50Hz(acX, acIdx);
  float gy = goertzel50Hz(acY, acIdx);
  float gz = goertzel50Hz(acZ, acIdx);
  float level = sqrtf(gx * gx + gy * gy + gz * gz);  // amplitudine vectoriala 50Hz

  // istoric scalat logaritmic: 0.02 .. 10 uT -> 0..40 px
  float frac = constrain((log10f(max(level, 0.005f)) - log10f(0.02f)) /
                         (log10f(10.0f) - log10f(0.02f)), 0.0f, 1.0f);
  hist[histIdx] = (uint8_t)(frac * 40);
  histIdx = (histIdx + 1) % HIST_N;

  tft.fillRect(0, 20, 128, 50, ST77XX_BLACK);
  tft.setTextSize(3);
  bool detect = level > 0.15f;
  tft.setTextColor(detect ? ST77XX_GREEN : ST77XX_WHITE);
  tft.setCursor(10, 28);
  if (level < 10) tft.printf("%.2f", level);
  else            tft.printf("%.1f", level);
  tft.setTextSize(1);
  tft.setTextColor(0x7BEF);
  tft.setCursor(92, 42);
  tft.print("uT AC");

  tft.fillRect(0, 60, 128, 10, ST77XX_BLACK);
  tft.setCursor(2, 60);
  tft.setTextColor(detect ? ST77XX_GREEN : 0x7BEF);
  // max 21 caractere pe linie (128px / 6px per caracter)
  tft.print(detect ? ">> CAMP 50Hz DETECTAT" : "cauta...consumator ON");

  // grafic istoric
  tft.fillRect(0, 74, 128, 42, 0x1082);
  for (int i = 0; i < HIST_N; i++) {
    int v = hist[(histIdx + i) % HIST_N];
    if (v > 0) tft.drawFastVLine(i, 115 - v, v, ST77XX_ORANGE);
  }
  tft.drawFastHLine(0, 115, 128, ST77XX_WHITE);

  tft.fillRect(0, 120, 128, 26, ST77XX_BLACK);
  tft.setCursor(2, 122);
  tft.setTextColor(0x7BEF);
  tft.printf("X:%.2f Y:%.2f Z:%.2f", gx, gy, gz);

  if (detect) tone(PIN_BUZZER, 300 + (int)(frac * 1200), 80);
}

// ---------------- Meteo ----------------
void uiMeteo() {
  tft.fillRect(0, 20, 128, 128, ST77XX_BLACK);
  if (!bmeOk) {
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(14, 70);
    tft.print("BME280 negasit!");
    return;
  }
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0f;
  // punct de roua (aproximatia Magnus)
  float g = logf(h / 100.0f) + 17.62f * t / (243.12f + t);
  float dew = 243.12f * g / (17.62f - g);

  tft.setTextSize(3);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(14, 28);
  tft.printf("%.1f%cC", t, 247);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(14, 62);
  tft.printf("%.0f %%RH", h);

  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(14, 88);
  tft.printf("%.1f", p);
  tft.setTextSize(1);
  tft.setCursor(98, 95);
  tft.print("hPa");

  tft.setTextColor(0x7BEF);
  tft.setCursor(14, 118);
  tft.printf("punct roua: %.1f%cC", dew, 247);
}

// ---------------- Calibrare busola ----------------
void runCalibration() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 30);  tft.print("CALIBRARE BUSOLA");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 50);  tft.print("Roteste placa in 8,");
  tft.setCursor(10, 62);  tft.print("pe toate axele, 20s");

  hmcConfig(0x18, 0x20);  // 75 Hz, gain 1.3 Ga
  float xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, zmin = 1e9, zmax = -1e9;
  uint32_t t0 = millis();
  while (millis() - t0 < 20000) {
    if (hmcDataReady()) {
      int16_t x, y, z;
      if (hmcRead(x, y, z) && !hmcSaturated(x, y, z)) {
        xmin = min(xmin, (float)x); xmax = max(xmax, (float)x);
        ymin = min(ymin, (float)y); ymax = max(ymax, (float)y);
        zmin = min(zmin, (float)z); zmax = max(zmax, (float)z);
      }
    }
    int w = (millis() - t0) * 108 / 20000;
    tft.drawRect(9, 90, 110, 10, ST77XX_WHITE);
    tft.fillRect(10, 91, w, 8, ST77XX_GREEN);
    delay(5);
  }

  offX = (xmin + xmax) / 2; offY = (ymin + ymax) / 2; offZ = (zmin + zmax) / 2;
  float rx = max((xmax - xmin) / 2, 1.0f);
  float ry = max((ymax - ymin) / 2, 1.0f);
  float rz = max((zmax - zmin) / 2, 1.0f);
  float ravg = (rx + ry + rz) / 3;
  sclX = ravg / rx; sclY = ravg / ry; sclZ = ravg / rz;

  prefs.putFloat("offX", offX); prefs.putFloat("offY", offY); prefs.putFloat("offZ", offZ);
  prefs.putFloat("sclX", sclX); prefs.putFloat("sclY", sclY); prefs.putFloat("sclZ", sclZ);

  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(38, 112);
  tft.print("SALVAT!");
  delay(1200);
  fInit = false;
  enterMode();
}

// ---------------- Comutare moduri ----------------
void enterMode() {
  noTone(PIN_BUZZER);
  tft.fillScreen(ST77XX_BLACK);
  drawHeader();
  lastNeedleAngle = -999;
  lastBubbleX = -999;
  fInit = false;
  acIdx = acCount = 0;

  switch (mode) {
    case MODE_BUSOLA:
      hmcConfig(0x70, 0x20);  // 8 medieri, 15 Hz, 1.3 Ga - stabil
      drawCompassFace();
      drawFooter("BOOT=mod  lung=calibrare");
      break;
    case MODE_MAGNET:
      hmcConfig(0x18, 0xE0);  // 75 Hz, gain 8.1 Ga - camp mare
      drawFooter("BOOT=mod urmator");
      break;
    case MODE_CABLU:
      hmcConfig(0x18, 0x20);  // 75 Hz, fara mediere, 1.3 Ga
      memset(hist, 0, sizeof(hist));
      histIdx = 0;
      drawFooter("porneste un consumator!");
      break;
    case MODE_METEO:
      drawFooter("BOOT=mod urmator");
      break;
    default: break;
  }
}

// ---------------- Buton ----------------
void handleButton() {
  bool down = digitalRead(PIN_BTN) == LOW || digitalRead(PIN_BTN_EXT) == LOW;
  uint32_t now = millis();
  if (down && !btnWasDown) {
    btnDownAt = now;
    longFired = false;
  } else if (down && !longFired && now - btnDownAt > 1500) {
    longFired = true;
    if (mode == MODE_BUSOLA) {
      tone(PIN_BUZZER, 1500, 150);
      runCalibration();
    }
  } else if (!down && btnWasDown && !longFired && now - btnDownAt > 30) {
    mode = (Mode)((mode + 1) % NUM_MODES);
    enterMode();
    tone(PIN_BUZZER, 800 + 150 * mode, 40);  // beep de confirmare, ton per mod
  }
  btnWasDown = down;
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_BTN_EXT, INPUT_PULLUP);
  pinMode(PIN_VBUS, INPUT);
  analogSetPinAttenuation(PIN_BAT_ADC, ADC_11db);  // pana la ~2.5V la pin
  pinMode(PIN_TFT_BLK, OUTPUT);
  digitalWrite(PIN_TFT_BLK, HIGH);  // backlight pornit

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  SPI.begin(PIN_TFT_SCK, -1, PIN_TFT_MOSI, PIN_TFT_CS);

  tft.initR(TAB_VARIANT);
  tft.setRotation(0);  // portret 128x160
  tft.fillScreen(ST77XX_BLACK);

  prefs.begin("magdet", false);
  offX = prefs.getFloat("offX", 0); offY = prefs.getFloat("offY", 0); offZ = prefs.getFloat("offZ", 0);
  sclX = prefs.getFloat("sclX", 1); sclY = prefs.getFloat("sclY", 1); sclZ = prefs.getFloat("sclZ", 1);

  bmeOk = bme.begin(0x77, &Wire);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 60);
  tft.print("MagDetector");
  tft.setTextSize(1);
  tft.setTextColor(0x7BEF);
  tft.setCursor(24, 84);
  tft.print("Carbon S2 v1.0");
  Serial.printf("MagDetector start. BME280: %s\n", bmeOk ? "OK" : "LIPSA");
  delay(1200);

  enterMode();
}

void loop() {
  handleButton();
  uint32_t now = millis();

  // esantionare magnetometru
  if (now - tSample >= 10 && (mode == MODE_BUSOLA || mode == MODE_MAGNET || mode == MODE_CABLU)) {
    if (hmcDataReady()) {
      tSample = now;
      int16_t x, y, z;
      if (hmcRead(x, y, z)) {
        if (mode == MODE_CABLU) {
          acX[acIdx] = x * curUtPerLsb;
          acY[acIdx] = y * curUtPerLsb;
          acZ[acIdx] = z * curUtPerLsb;
          acIdx = (acIdx + 1) % AC_N;
          if (acCount < AC_N) acCount++;
        } else if (mode == MODE_BUSOLA) {
          if (!hmcSaturated(x, y, z)) {
            float mx = (x - offX) * sclX;
            float my = (y - offY) * sclY;
            float mzc = (z - offZ) * sclZ;
            if (now - tUi >= 100) { tUi = now; uiBusola(mx, my, mzc); }
          }
        } else {  // MODE_MAGNET
          if (now - tUi >= 200) {
            tUi = now;
            uiMagnet(x, y, z, hmcSaturated(x, y, z));
          }
        }
      }
    }
  }

  if (mode == MODE_CABLU && now - tUi >= 500) { tUi = now; uiCablu(); }
  if (mode == MODE_METEO && now - tUi >= 1000) { tUi = now; uiMeteo(); }
  if (now - tBat >= 5000) { tBat = now; drawBattery(); }
}
