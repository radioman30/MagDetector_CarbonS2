// ============================================================
//  Carcasa MagDetector - Carbon S2 + ST7735 1.8" + GY-271 + BME280
//  Doua piese: baza (tava) si capac (cu fereastra display + buton)
//  Sonda magnetometru in "ciocul" din fata, departe de electronica.
//
//  Randare: part = "base" | "lid" | "all" (previzualizare asamblata)
//  Toate cotele sunt parametrice - ajusteaza si regenereaza.
// ============================================================

part = "all";

$fn = 48;

/* ---------------- Parametri generali ---------------- */
wall      = 2.0;    // grosime pereti
floor_t   = 2.0;    // grosime podea / placa capac
body_w    = 46;     // latime exterioara corp
body_l    = 130;    // lungime exterioara corp
corner_r  = 5;      // raza colturi corp
nose_w    = 24;     // latime cioc sonda
nose_l    = 22;     // lungime cioc (dincolo de corp)
nose_r    = 4;      // raza colturi cioc
base_h    = 18;     // inaltime exterioara baza
lid_h     = 12;     // inaltime exterioara capac
lip_h     = 3;      // buza de imbinare capac->baza
clr       = 0.20;   // joc de imbinare

/* ---------------- Display ST7735 1.8" ---------------- */
disp_cy      = 90;     // centrul modulului pe axa Y
disp_hole_dx = 32.7;   // distanta gauri montaj pe X (masurat centru-centru)
disp_hole_dy = 56.8;   // distanta gauri montaj pe Y (masurat centru-centru)
disp_boss_d  = 6;      // diametru bosaje
disp_boss_h  = 3.5;    // inaltime bosaje (aduce sticla la fata)
disp_pilot   = 1.8;    // pilot M2 autofiletant
win_w        = 35;     // fereastra (sticla 34.5 + 0.5 joc)
win_l        = 47;     // fereastra (sticla 46.5 + 0.5 joc)
win_off_y    = 0;      // ajustare fina fereastra fata de centru modul

/* ---------------- Buton / buzzer / senzori ---------------- */
btn_pos   = [0, 50];   // centrul gaurii de buton pe capac (sub display, deasupra bateriei)
btn_d     = 12.4;      // gaura buton panou 12mm
buz_pos   = [0, 120];  // buzzer pe podea baza
buz_d     = 13;        // diametru locas buzzer (disc 12mm)
bme_vent_y = [70, 86]; // zona fantelor de aerisire BME280 (perete dreapta)
usb_cy    = 103;       // centrul decupajului USB-C (perete dreapta)
usb_w     = 10;        // latime decupaj USB
usb_h     = 5;         // inaltime decupaj USB
gy_pock   = [16, 15];  // buzunar GY-271 in cioc (X x Y)

/* ---------------- Baterie LiPo ---------------- */
bat_bay   = [36, 54];  // compartiment baterie (X x Y)
bat_y0    = 14;        // inceputul compartimentului pe Y
bay_wall  = 1.5;
bay_h     = 6;

/* ---------------- Suruburi carcasa (M3) ---------------- */
post_d    = 8;
post_pos  = [[-15, 7], [15, 7], [-15, 124], [15, 124]];
screw_thru = 3.4;      // gaura libera M3 in baza
screw_pilot = 2.7;     // pilot M3 in capac
csink_d   = 6.5;       // cap surub ingropat

/* ---------------- Carbon S2 ---------------- */
pcb_l = 37.5;          // lungime placa (transversal, USB spre dreapta)
pcb_w = 18.5;
pcb_cy = disp_cy + 13; // sub jumatatea de sus a display-ului

// ============================================================
//  Conturul carcasei (corp + cioc, unite prin hull -> taper)
// ============================================================
module case_outline(o = 0) {
  offset(r = o) hull() {
    offset(r = corner_r) offset(delta = -corner_r)
      translate([-body_w/2, 0]) square([body_w, body_l]);
    offset(r = nose_r) offset(delta = -nose_r)
      translate([-nose_w/2, body_l - 1]) square([nose_w, nose_l + 1]);
  }
}

// ============================================================
//  BAZA
// ============================================================
module base() {
  difference() {
    union() {
      // podea
      linear_extrude(floor_t) case_outline();
      // pereti
      linear_extrude(base_h)
        difference() { case_outline(); case_outline(-wall); }

      // compartiment baterie
      translate([0, bat_y0 + bat_bay[1]/2, 0]) linear_extrude(floor_t + bay_h)
        difference() {
          square([bat_bay[0] + 2*bay_wall, bat_bay[1] + 2*bay_wall], center = true);
          square([bat_bay[0], bat_bay[1]], center = true);
          // ferestre laterale pt banda velcro / degete
          square([bat_bay[0] + 4*bay_wall, 20], center = true);
        }

      // ghidaje colturi Carbon S2 (placa transversala, USB spre dreapta)
      for (sx = [-1, 1], sy = [-1, 1])
        translate([sx*(pcb_l/2 + 0.25), pcb_cy + sy*(pcb_w/2 + 0.25), floor_t])
          linear_extrude(3) difference() {
            square([6, 6], center = true);
            translate([-sx*3, -sy*3]) square([6, 6], center = true);
          }

      // inel locas buzzer
      translate([buz_pos[0], buz_pos[1], 0]) linear_extrude(floor_t + 4)
        difference() { circle(d = buz_d + 3); circle(d = buz_d); }

      // buzunar GY-271 in cioc (3 pereti, deschis spre corp pt fire)
      translate([0, body_l + nose_l/2 + 1, floor_t]) linear_extrude(4)
        difference() {
          square([gy_pock[0] + 3, gy_pock[1] + 3], center = true);
          square([gy_pock[0], gy_pock[1]], center = true);
          translate([0, -gy_pock[1]/2 - 3]) square([gy_pock[0] - 4, 8], center = true);
        }

      // coloane suruburi (se opresc sub buza capacului)
      for (p = post_pos)
        translate([p[0], p[1], 0]) cylinder(d = post_d, h = base_h - lip_h - 0.2);
    }

    // gauri suruburi + cap ingropat
    for (p = post_pos) translate([p[0], p[1], -0.01]) {
      cylinder(d = screw_thru, h = base_h + 1);
      cylinder(d = csink_d, h = 1.8);
    }

    // decupaj USB-C, perete dreapta
    translate([body_w/2 - wall - 0.5, usb_cy - usb_w/2, floor_t + 1.2])
      cube([wall + 1.5, usb_w, usb_h]);

    // fante aerisire BME280, perete dreapta
    for (y = [bme_vent_y[0] : 5 : bme_vent_y[1]])
      translate([body_w/2 - wall - 0.5, y, floor_t + 3])
        cube([wall + 1.5, 2, 8]);

    // gauri sunet buzzer, in podea
    translate([buz_pos[0], buz_pos[1], -0.01]) {
      cylinder(d = 2, h = floor_t + 1);
      for (a = [0:60:300]) rotate([0, 0, a])
        translate([3.5, 0, 0]) cylinder(d = 2, h = floor_t + 1);
    }
  }
}

// ============================================================
//  CAPAC (modelat in orientarea asamblata; z=0 la marginea de jos)
// ============================================================
module lid() {
  difference() {
    union() {
      // placa superioara
      translate([0, 0, lid_h - floor_t]) linear_extrude(floor_t) case_outline();
      // fusta
      linear_extrude(lid_h)
        difference() { case_outline(); case_outline(-wall); }
      // buza care intra in baza
      translate([0, 0, -lip_h]) linear_extrude(lip_h + 0.01)
        difference() { case_outline(-wall - clr); case_outline(-wall - clr - 1.4); }

      // bosaje montaj display (M2)
      for (sx = [-1, 1], sy = [-1, 1])
        translate([sx*disp_hole_dx/2, disp_cy + sy*disp_hole_dy/2, lid_h - floor_t - disp_boss_h])
          cylinder(d = disp_boss_d, h = disp_boss_h);

      // coloane primire suruburi M3
      for (p = post_pos)
        translate([p[0], p[1], -lip_h])
          cylinder(d = post_d, h = lid_h - floor_t + lip_h);
    }

    // fereastra display
    translate([0, disp_cy + win_off_y, lid_h - floor_t - 0.5])
      linear_extrude(floor_t + 1)
        square([win_w, win_l], center = true);

    // gaura buton
    translate([btn_pos[0], btn_pos[1], lid_h - floor_t - 0.5])
      cylinder(d = btn_d, h = floor_t + 1);

    // piloti M2 display
    for (sx = [-1, 1], sy = [-1, 1])
      translate([sx*disp_hole_dx/2, disp_cy + sy*disp_hole_dy/2, lid_h - floor_t - disp_boss_h - 0.5])
        cylinder(d = disp_pilot, h = disp_boss_h + 0.4);

    // piloti M3 capac
    for (p = post_pos)
      translate([p[0], p[1], -lip_h - 0.01])
        cylinder(d = screw_pilot, h = 9);

    // text gravat
    translate([0, 30, lid_h - 0.6])
      linear_extrude(1)
        text("MagDetector", size = 6, halign = "center", valign = "center");
  }
}

// ============================================================
//  Selectie piesa
// ============================================================
if (part == "base") base();
if (part == "lid")  rotate([180, 0, 0]) translate([0, 0, -lid_h]) lid();  // rasturnat pt printare
if (part == "all") {
  base();
  color("SteelBlue", 0.65) translate([0, 0, base_h]) lid();
}
