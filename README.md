# MagDetector — Carbon S2

Instrument portabil 4-în-1 pe GroundStudio Carbon S2 (ESP32-S2FN4R2):
**busolă** · **detector poli magnet** · **detector cabluri 50 Hz** · **mini stație meteo**

## Hardware

| Componentă | Conexiune |
|---|---|
| HMC5883L (magnetometru, original Honeywell, 0x1E) | I2C: SDA=GPIO8, SCL=GPIO9 |
| BME280 (temp/umiditate/presiune, 0x77) | același I2C |
| TFT ST7735 1.8" 128×160 | SCK=36, MOSI=35, CS=37, DC=38, RST=33, BLK=14 |
| Buton moduri | BOOT (GPIO0) **sau** buton extern GPIO4→GND |
| Buzzer pasiv | GPIO5→GND |
| Baterie LiPo 1S | conectorul plăcii; măsurare internă pe GPIO3, VBUS pe GPIO21 |

**Atenție GPIO3**: e nod triplu (divizor 442k/160k + pinul CHRG al charger-ului
BRCL4054 + LED-ul de charging). Tensiunea bateriei e validă **doar fără USB**;
la încărcare pinul e tras la ~0V (firmware-ul afișează "se incarca").
`BAT_DIV = 3.790` calibrat cu multimetrul (teoretic 3.7625).

## Utilizare

- **Apăsare scurtă** buton = următorul mod (beep cu ton diferit per mod)
- **Apăsare lungă (1.5s) în modul Busolă** = calibrare hard-iron: rotește în 8
  pe toate axele 20s; se salvează în NVS, se face o singură dată
- **Cabluri**: funcționează doar pe cabluri prin care TRECE CURENT — pornește
  un consumator mare (aerotermă 2kW) pe circuit. Detecție prin filtru Goertzel
  pe 50 Hz (aliazat la 25 Hz, fs=75 Hz, N=96), prag 0.15 µT
- **Magnet**: polaritate pe axa Z (semn inversat, verificat cu busolă reală);
  la "SATURAT" depărtează magnetul

## Compilare & upload

```
arduino-cli (din C:\Users\123\ArduinoCompiler\arduino-cli\)
FQBN: esp32:esp32:esp32s2:CDCOnBoot=cdc,FlashSize=4M,PSRAM=enabled
```

Portul COM se re-enumerează la fiecare reset (COM24–27...) — refă
`arduino-cli board list` înainte de upload; de obicei merge din a 2-a încercare.
Biblioteci: Adafruit GFX + ST7735 + BME280 (instalate).

## Structură

- `MagDetector/` — firmware-ul principal (v1.1, cu Goertzel pe modul cabluri)
- `I2C_Scan/` — scanner I2C cu autodetecție pini și identificare chip-uri
- `carcasa/` — carcasă parametrică OpenSCAD + STL-uri gata de print:
  - `magdetector_baza.stl`, `magdetector_capac.stl` (46×152×26 mm)
  - PLA/PETG, 0.2 mm, 3 perimetre, fără suporturi, ambele piese flat pe pat
  - cioc-sondă în față pt. GY-271 (departe de electronică!), fereastră TFT,
    buton pe capac, USB-C lateral dreapta, compartiment LiPo 36×54×10
  - de verificat înainte de print: găurile modulului de display la
    29.5×51.5 mm (`disp_hole_dx/dy`), dimensiunea bateriei (`bat_bay`)
  - asamblare: 4×M3×20-25 prin podea + 4×M2×5 pt display
