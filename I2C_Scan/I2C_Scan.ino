// I2C Scanner pentru GroundStudio Carbon S2 (ESP32-S2FN4R2)
// Scaneaza mai multe perechi SDA/SCL candidate si identifica chip-urile gasite.
// Proiect: MagDetector (busola + poli magnet + detector cabluri)

#include <Wire.h>

// Perechi candidate {SDA, SCL}. Evitate: 0 (boot), 19/20 (USB), 26 (PSRAM), 45/46 (strapping)
const int pinPairs[][2] = {
  {8, 9},   {9, 8},
  {1, 2},   {2, 1},
  {3, 4},   {4, 3},
  {5, 6},   {6, 5},
  {7, 10},  {10, 7},
  {11, 12}, {12, 11},
  {13, 14}, {14, 13},
  {15, 16}, {16, 15},
  {17, 18}, {18, 17},
  {21, 33}, {33, 21},
  {33, 34}, {34, 33},
  {35, 36}, {36, 35},
  {37, 38}, {38, 37},
};
const int numPairs = sizeof(pinPairs) / sizeof(pinPairs[0]);

const char* identify(uint8_t addr) {
  switch (addr) {
    case 0x0D: return "QMC5883L (magnetometru, clona)";
    case 0x1E: return "HMC5883L (magnetometru, original)";
    case 0x76: return "BMx280 (temp/presiune, SDO=GND)";
    case 0x77: return "BMx280 (temp/presiune, SDO=VCC)";
    default:   return "necunoscut";
  }
}

// Pentru BMx280 citim registrul de chip ID (0xD0) ca sa stim exact ce e
void checkBmx280(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0xD0);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((int)addr, 1) != 1) return;
  uint8_t id = Wire.read();
  Serial.printf("      chip ID 0x%02X -> %s\n", id,
                id == 0x60 ? "BME280 (are si umiditate!)" :
                id == 0x58 ? "BMP280 (doar temp+presiune)" :
                id == 0x61 ? "BME680" : "ID necunoscut");
}

int scanPair(int sda, int scl, bool verbose) {
  Wire.end();
  if (!Wire.begin(sda, scl, 100000)) return 0;
  Wire.setTimeOut(50);
  int found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found++;
      if (verbose) {
        Serial.printf("    0x%02X - %s\n", addr, identify(addr));
        if (addr == 0x76 || addr == 0x77) checkBmx280(addr);
      }
    }
  }
  return found;
}

void setup() {
  Serial.begin(115200);
  delay(3000);  // timp pentru re-enumerare USB CDC
  Serial.println("\n=== I2C Scanner - Carbon S2 ===");
}

void loop() {
  bool anyFound = false;
  for (int i = 0; i < numPairs; i++) {
    int sda = pinPairs[i][0], scl = pinPairs[i][1];
    int n = scanPair(sda, scl, false);
    if (n > 0) {
      anyFound = true;
      Serial.printf("\n>>> GASIT pe SDA=GPIO%d SCL=GPIO%d (%d dispozitive):\n", sda, scl, n);
      scanPair(sda, scl, true);
    }
  }
  if (!anyFound)
    Serial.println("Nimic gasit. Verifica firele (VCC, GND, SDA, SCL) si reia.");
  Serial.println("\n--- rescan in 5s ---");
  delay(5000);
}
