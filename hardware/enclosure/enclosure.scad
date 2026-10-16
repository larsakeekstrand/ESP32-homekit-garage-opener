// enclosure.scad — garage-opener rev 1 enclosure.
// part = "base" | "lid" | "btn_boot" | "btn_en" | "lightpipe" | "fittest" | "template" | "assembly"
// All dimensions mm; coordinates as in board_dims.scad (origin = board bottom-left on its underside, Y up, Z to the lid).
// Parts are modelled in place (assembled position); turn them for printing in the slicer (see README).
include <params.scad>
include <devkit_dims.scad>
include <board_dims.scad>
part = "assembly";

// ---- derived ----
cav_x0 = -slip;  cav_x1 = pcb_w + slip;           // cavity = board + slip all round
cav_y0 = -slip;  cav_y1 = pcb_h + slip;
board_z = 0;
tall = max([for (r = z_max_by_region) r[5]]);
lip_h = 2;                                        // the lid's locating lip inside the base walls
lid_inner_z = pcb_t + max(tall, stack_h) + 0.5 + lip_h;   // underside of the lid = top of the base walls
floor_z = -standoff - floor_t;                    // underside of the floor
out_x0 = cav_x0 - wall; out_x1 = cav_x1 + wall; out_y0 = cav_y0 - wall; out_y1 = cav_y1 + wall;
slab_d = 4;                                       // solid back slab: the base prints lying on it, no supports
back_z = floor_z - slab_d;                        // back face of the box (against the wall)
usb_y = u1_pin19[1] + usb_dy; usb_cz = pcb_t + usb_z;
sma_y = u1_pin1[1] - 1; sma_z = pcb_t + 6.5;      // clear of the board edge and of the H2 lid tube
// Keyholes: the box hangs on two wall screws. The head goes in through the round hole, then the box drops so the
// neck (a narrow slot next to the wall) slides under the head; the head sits in a wider channel behind the neck.
// Printed back-down, the channel only steps outwards above the neck, so nothing overhangs.
kh_hole = wall_screw_head_d + 1; kh_slot = wall_screw_d + 0.5; kh_neck = 2; kh_travel = 10;
kh_xs = [out_x0 + 18, out_x1 - 18];
kh_rest = out_y1 - 12;                            // where the screws end up: the drilling template's holes
kh_y = kh_rest - kh_travel;                       // the round holes
back_top = floor_z + floor_t;                     // top of the back material (slab + floor)

module rbox(x0, y0, x1, y1, z0, z1, r = corner_r) {
  translate([0, 0, z0]) linear_extrude(z1 - z0) hull()
    for (x = [x0 + r, x1 - r], y = [y0 + r, y1 - r]) translate([x, y]) circle(r);
}
plug_margin = 1.5;                                // the mating plugs are wider/taller than the headers
function conn_cut(c) = [c[1] - plug_margin, c[2] + plug_margin, pcb_t - 0.5, pcb_t + c[3] + 3];   // x0, x1, z0, z1

module base() {
  difference() {
    rbox(out_x0, out_y0, out_x1, out_y1, back_z, lid_inner_z);
    rbox(cav_x0, cav_y0, cav_x1, cav_y1, floor_z + floor_t, lid_inner_z + 1, 1);
    // connector cut-outs through the bottom wall (the plugs go in from below)
    for (c = conn) let(k = conn_cut(c)) translate([k[0], out_y0 - 1, k[2]]) cube([k[1] - k[0], wall + 2, k[3] - k[2]]);
    // USB through the left wall
    translate([out_x0 - 1, usb_y - usb_plug_w / 2, usb_cz - usb_plug_h / 2]) cube([wall + 2, usb_plug_w, usb_plug_h]);
    // SMA bulkhead D-hole through the right wall (the flat stops the bulkhead turning)
    translate([out_x1 - wall - 1, sma_y, sma_z]) rotate([0, 90, 0]) linear_extrude(wall + 2)
      intersection() { circle(d = 6.5); square([6.5, 5.8], center = true); }
    // DHT vents in the bottom and right walls, beside the sensor
    for (i = [0:5]) translate([dht[0] + 2 + i * 2.2, out_y0 - 1, floor_z + floor_t + 2]) cube([1.2, wall + 2, 12]);
    for (i = [0:5]) translate([out_x1 - wall - 1, dht[1] + 2 + i * 2.2, floor_z + floor_t + 2]) cube([wall + 2, 1.2, 12]);
    // keyholes: round hole through the back, neck slot next to the wall, head channel behind it (open to the cavity)
    for (x = kh_xs) {
      translate([x, kh_y, back_z - 0.01]) cylinder(d = kh_hole, h = back_top - back_z + 0.02);
      hull() for (y = [kh_y, kh_rest]) translate([x, y, back_z - 0.01]) cylinder(d = kh_slot, h = kh_neck + 0.02);
      hull() for (y = [kh_y, kh_rest]) translate([x, y, back_z + kh_neck]) cylinder(d = kh_hole, h = back_top - back_z - kh_neck + 0.01);
    }
    // heat-set insert holes in the PCB bosses (the lid screws clamp the PCB onto them)
    for (h = holes) translate([h[0], h[1], -insert_h]) cylinder(d = insert_d, h = insert_h + 0.01);
  }
  for (h = holes) difference() {
    translate([h[0], h[1], floor_z + floor_t - 0.01]) cylinder(d = 7, h = standoff + 0.01);
    translate([h[0], h[1], -insert_h]) cylinder(d = insert_d, h = insert_h + 0.02);
  }
  // DHT baffle under the board, along the keep-out's top and left edges
  translate([dht[0] - 1.2, dht[1], floor_z + floor_t - 0.01]) cube([1.2, dht[3] - dht[1], standoff - 0.3]);
  translate([dht[0] - 1.2, dht[3], floor_z + floor_t - 0.01]) cube([dht[2] - dht[0] + 1.2, 1.2, standoff - 0.3]);
  echo("FIT", "cav_x0", cav_x0); echo("FIT", "cav_x1", cav_x1); echo("FIT", "cav_y0", cav_y0); echo("FIT", "cav_y1", cav_y1);
  echo("FIT", "board_z", board_z); echo("FIT", "lid_inner_z", lid_inner_z); 
  echo("FIT", "bosses", [for (h = holes) [h[0], h[1]]]);
  echo("FIT", "out", [out_x0, out_y0, out_x1, out_y1]); echo("FIT", "back_slab", [out_x0, out_y0, out_x1, out_y1, back_z]);
  echo("FIT", "back", [back_z, back_top]);
  echo("FIT", "keyholes", [for (x = kh_xs) [x, kh_y, kh_rest]]);
  echo("FIT", "kh_dims", [kh_hole, kh_slot, kh_neck, back_top - back_z - kh_neck]);
  echo("FIT", "usb_cut", [usb_y - usb_plug_w / 2, usb_y + usb_plug_w / 2, usb_cz - usb_plug_h / 2, usb_cz + usb_plug_h / 2]);
  for (c = conn) echo("FIT", str("conn_cut_", c[0]), conn_cut(c));
}

// Drilling template: the box's outline with a 2.5 mm hole where each screw goes. Hold it level where the box will
// hang, mark through the holes, drill. Prints flat in a few minutes.
module template() {
  difference() {
    rbox(out_x0, out_y0, out_x1, out_y1, 0, 1.2);
    for (x = kh_xs) translate([x, kh_rest, -1]) cylinder(d = 2.5, h = 3);
    translate([(out_x0 + out_x1) / 2, out_y0 + 12, -1]) cylinder(d = 20, h = 3);   // saves plastic
  }
  translate([(out_x0 + out_x1) / 2, out_y1 - 14, 1.19]) linear_extrude(0.6) text("UP", size = 6, halign = "center", valign = "center");
  echo("FIT", "holes", [for (x = kh_xs) [x, kh_rest]]);
}
// The wall screws, for the reference models only.
module wall_screws() {
  color("silver") for (x = kh_xs) translate([x, kh_rest, back_z - 20]) {
    cylinder(d = wall_screw_d, h = 20 + kh_neck + 0.3);
    translate([0, 0, 20 + kh_neck + 0.3]) cylinder(d = wall_screw_head_d, h = wall_screw_head_h);
  }
}

// ---- lid and small parts ----
led_tube_od = 5.2; led_hole = 3.2; led_dome_z = pcb_t + 5.3;   // LED_D3.0mm sits ~5.3 mm tall
tube_od = 6.5; screw_d = 3.4;                     // lid tubes over H1..H4: M3 countersunk screws into the bosses
head_sink = lid_inner_z + lid_t - lid_screw_len + 4.0;   // counterbore so the screw tip lands at z = -4 (4 mm into the insert)
head_od = 9;                                      // the tubes thicken near the lid to hold the sunk heads
btn_d = 4.4; btn_sleeve_od = 7; btn_rest_gap = 0.5; en_recess = 1.5;
sleeve_bottom_z = pcb_t + stack_h + 0.5;          // the sleeves stay above the devkit's tallest part
lid_top_z = lid_inner_z + lid_t;
function btn_xy(o) = [u1_pin19[0] + o[0], u1_pin19[1] + o[1]];

module label(t, x, y, s = 4) {                    // debossed 0.6 mm into the lid's top face, readable from outside
  translate([x, y, lid_top_z - 0.6]) linear_extrude(0.61)
    text(t, size = s, halign = "center", valign = "center", font = "Liberation Sans:style=Bold");
}

module lid() {
  difference() {
    union() {
      rbox(out_x0, out_y0, out_x1, out_y1, lid_inner_z, lid_top_z);
      // locating lip just inside the base walls
      difference() {
        rbox(cav_x0 + slip, cav_y0 + slip, cav_x1 - slip, cav_y1 - slip, lid_inner_z - lip_h, lid_inner_z + 0.01, 1);
        rbox(cav_x0 + slip + 1.6, cav_y0 + slip + 1.6, cav_x1 - slip - 1.6, cav_y1 - slip - 1.6, lid_inner_z - 2.1, lid_inner_z + 0.1, 1);
      }
      for (h = holes) translate([h[0], h[1], pcb_t]) cylinder(d = tube_od, h = lid_inner_z - pcb_t + 0.01);
      for (h = holes) translate([h[0], h[1], lid_top_z - head_sink - 1.7 - 1.5]) cylinder(d = head_od, h = head_sink + 1.7 + 1.5);
      for (l = leds) translate([l[1], l[2], led_dome_z + 1]) cylinder(d = led_tube_od, h = lid_inner_z - led_dome_z - 1 + 0.01);
      for (o = [btn_boot, btn_en]) let(q = btn_xy(o)) translate([q[0], q[1], sleeve_bottom_z]) cylinder(d = btn_sleeve_od, h = lid_inner_z - sleeve_bottom_z + 0.01);
      // DHT baffle from above, along the keep-out's top and left edges, stopping 1 mm over the flat sensor (~9 mm)
      translate([dht[0] - 1.2, dht[1], pcb_t + 10]) cube([1.2, dht[3] - dht[1], lid_inner_z - pcb_t - 10 + 0.01]);
      translate([dht[0] - 1.2, dht[3], pcb_t + 10]) cube([dht[2] - dht[0] + 1.2, 1.2, lid_inner_z - pcb_t - 10 + 0.01]);
    }
    for (h = holes) translate([h[0], h[1], pcb_t - 1]) {
      cylinder(d = screw_d, h = 50);
      translate([0, 0, lid_top_z - pcb_t + 1 - head_sink - 1.7]) { cylinder(d1 = screw_d, d2 = 6.6, h = 1.71); translate([0, 0, 1.7]) cylinder(d = 6.8, h = 30); }
    }
    for (l = leds) translate([l[1], l[2], 0]) cylinder(d = led_hole, h = 50);
    for (o = [btn_boot, btn_en]) let(q = btn_xy(o)) translate([q[0], q[1], 0]) cylinder(d = btn_d + 2 * slip, h = 50);
    let(q = btn_xy(btn_en)) translate([q[0], q[1], lid_top_z - en_recess]) cylinder(d = 9, h = 2);   // EN can't be hit by accident
    for (i = [0:5]) translate([dht[0] + 2 + i * 2.2, dht[1] + 3, lid_inner_z - 1]) cube([1.2, dht[3] - dht[1] - 6, lid_t + 2]);
    for (i = [0 : len(leds) - 1]) label(leds[i][0], leds[i][1], leds[i][2] + (i == 0 ? -5.5 : 5.5), 3);   // RUN below, DOOR above
    let(q = btn_xy(btn_boot)) label("BOOT", q[0] + 10, q[1], 3);
    let(q = btn_xy(btn_en)) label("RST", q[0] + 10.5, q[1], 3);
    label("GARAGE", (out_x0 + out_x1) / 2 + 10, out_y1 - 10, 7);
  }
  echo("FIT", "lid_bottom_z", lid_inner_z); echo("FIT", "lip_bottom_z", lid_inner_z - lip_h);
  echo("FIT", "led_holes", [for (l = leds) [l[1], l[2]]]);
  echo("FIT", "btn_holes", [btn_xy(btn_boot), btn_xy(btn_en)]);
  echo("FIT", "tubes", [for (h = holes) [h[0], h[1]]]);
  echo("FIT", "tube_bottom_z", pcb_t); echo("FIT", "screw_tip_z", lid_top_z - head_sink - lid_screw_len);
  echo("FIT", "sleeve_bottom_z", sleeve_bottom_z);
}

// Push-rod, modelled at rest: tip 0.5 mm above the switch, cap 1.5 mm proud of the lid face (EN: of its recess
// floor) so a press can travel through the gap and the switch. Inserted from outside; the cone below the sleeve
// snaps through and keeps it captive, the slot lets it squeeze.
module button(o, recess = 0) {
  q = btn_xy(o); z0 = pcb_t + btn_top_z + btn_rest_gap; face_z = lid_top_z - recess; cap_z = face_z + btn_rest_gap + 1.0;
  translate([q[0], q[1], 0]) difference() {
    union() {
      translate([0, 0, z0]) cylinder(d = btn_d, h = cap_z - z0);
      translate([0, 0, cap_z]) cylinder(d = btn_d + 2, h = 1.2);
      translate([0, 0, sleeve_bottom_z - 0.2 - 0.8]) cylinder(d1 = btn_d, d2 = btn_d + 1.2, h = 0.8);
    }
    translate([-0.5, -5, z0 - 0.01]) cube([1, 10, sleeve_bottom_z - z0 + 0.5]);
  }
  echo("FIT", "rod_bottom_z", z0); echo("FIT", "cap_bottom_z", cap_z); echo("FIT", "face_z", face_z);
}
module lightpipe() { cylinder(d = led_hole - 0.2, h = lid_t + 4); translate([0, 0, lid_t + 4]) cylinder(d = led_hole + 1.5, h = 0.8); }
module fittest() {   // the bottom 16 mm of the base (all three plug cut-outs) and the back around one keyhole: plug in,
                     // and hang the coupon on a screw
  intersection() { base(); translate([out_x0 - 1, out_y0 - 1, back_z - 1]) cube([out_x1 - out_x0 + 2, 17, lid_inner_z - back_z + 2]); }
  intersection() { base(); translate([kh_xs[0] - kh_hole / 2 - 5, kh_y - kh_hole / 2 - 5, back_z - 1]) cube([kh_hole + 10, kh_travel + kh_hole + 10, back_top - back_z + 1]); }
}

// ---- for the assembly render only ----
// The PCB with its components, from kicad-cli's STL export (page coords: X = page X, Y = -page Y, Z = 0 at the
// board underside; the board's top-left corner is at page (100, 100)), plus a stand-in for the devkit, which
// has no 3D model: two sockets, its PCB, the WROOM-32U module and the USB receptacle.
module pcb_model() {
  color("green") translate([-100, 100 + pcb_h, 0]) import("../garage-opener-pcb/fab/garage-opener-pcb.stl");
  dk_x0 = u1_pin19[0] - 3.18; dk_y1 = u1_pin1[1] + 1.25; dk_len = 54.4; dk_w = 27.9;
  color("black") for (y = [u1_pin1[1], u1_pin1[1] - 25.4]) translate([u1_pin19[0] - 1.27, y - 1.27, pcb_t]) cube([u1_pin1[0] - u1_pin19[0] + 2.54, 2.54, seat_h]);
  translate([dk_x0, dk_y1 - dk_w, pcb_t + seat_h]) {
    color("royalblue") cube([dk_len, dk_w, devkit_t]);
    color("silver") translate([dk_len - 19.2, (dk_w - 18) / 2, devkit_t]) cube([18.4, 18, 3.2]);          // WROOM-32U
    color("silver") translate([0, u1_pin19[1] + usb_dy - (dk_y1 - dk_w) - usb_w / 2, devkit_t]) cube([5.5, usb_w, usb_hgt]);
    for (o = [btn_boot, btn_en]) color("dimgray") translate([u1_pin19[0] + o[0] - dk_x0 - 1.5, u1_pin19[1] + o[1] - (dk_y1 - dk_w) - 2, devkit_t]) cube([3, 4, btn_top_z - seat_h - devkit_t]);
  }
}

// Every piece in place (e = 0), or pulled apart along the way it assembles (e = 1): the screws stay in the wall,
// the base hangs on them from above, the PCB drops into the base, the lid and its buttons come down on top.
module everything(e) {
  wall_screws();
  translate([0, e * kh_travel, e * 15]) {
    base();
    translate([0, 0, e * 30]) pcb_model();
    translate([0, 0, e * 75]) { lid(); button(btn_boot); button(btn_en, en_recess); }
    translate([0, 0, e * 95]) for (l = leds) translate([l[1], l[2], lid_top_z - lid_t - 4]) lightpipe();
  }
}

if (part == "base") base();
else if (part == "lid") lid();
else if (part == "btn_boot") button(btn_boot);
else if (part == "btn_en") button(btn_en, en_recess);
else if (part == "lightpipe") lightpipe();
else if (part == "fittest") fittest();
else if (part == "template") template();
else if (part == "assembled") everything(0);           // reference only, not for printing
else if (part == "exploded") everything(1);             // reference only, not for printing
else if (part == "assembly") {
  wall_screws(); color("white", 0.7) base(); pcb_model();
  color("white", 0.5) translate([0, 0, 25]) lid();   // lid lifted 25 mm to show the inside
  color("orange") { translate([0, 0, 25]) { button(btn_boot); button(btn_en, en_recess); } }
}
