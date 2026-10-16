// enclosure.scad — garage-opener rev 1 enclosure.
// part = "base" | "bracket" | "lid" | "btn_boot" | "btn_en" | "lightpipe" | "fittest" | "assembly"
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
lid_inner_z = pcb_t + max(tall, stack_h) + 1.5;  // underside of the lid = top of the base walls
floor_z = -standoff - floor_t;                    // underside of the floor
out_x0 = cav_x0 - wall; out_x1 = cav_x1 + wall; out_y0 = cav_y0 - wall; out_y1 = cav_y1 + wall;
groove_mouth = 10; groove_d = 3.2;                // dovetail: narrow mouth at the back face, 45 deg flanks
housing_d = 4; housing_w = groove_mouth + 2 * groove_d + 4;
back_z = floor_z - housing_d;                     // back face of the box (touches the bracket's front face)
groove_xs = [out_x0 + 18, out_x1 - 18];
usb_y = u1_pin19[1] + usb_dy; usb_cz = pcb_t + usb_z;
sma_y = u1_pin1[1] - 1; sma_z = pcb_t + 6.5;      // clear of the board edge and of the H2 lid tube
latch_x = (out_x0 + out_x1) / 2;
latch_y0 = out_y1 - 14;                           // latch block on the back, top centre
pocket_y0 = out_y1 - 12.3; pocket_len = 3.6;

module rbox(x0, y0, x1, y1, z0, z1, r = corner_r) {
  translate([0, 0, z0]) linear_extrude(z1 - z0) hull()
    for (x = [x0 + r, x1 - r], y = [y0 + r, y1 - r]) translate([x, y]) circle(r);
}
// Extrudes a 2D profile drawn in (x, z) along +Y from y = 0 to y = len.
module prism_y(len) { translate([0, len, 0]) rotate([90, 0, 0]) linear_extrude(len) children(); }
// Dovetail cross-section: width w at z = 0, widening 45 deg to w + 2h at z = h.
module dovetail2d(w, h) { polygon([[-w / 2, 0], [w / 2, 0], [w / 2 + h, h], [-w / 2 - h, h]]); }
function conn_cut(c) = [c[1] - slip, c[2] + slip, pcb_t - 0.5, pcb_t + c[3] + slip];   // x0, x1, z0, z1

module base() {
  difference() {
    union() {
      rbox(out_x0, out_y0, out_x1, out_y1, floor_z, lid_inner_z);
      for (gx = groove_xs) translate([gx - housing_w / 2, out_y0, back_z]) cube([housing_w, out_y1 - out_y0, housing_d + 0.01]);
      translate([latch_x - 7, latch_y0, back_z]) cube([14, 10, housing_d + 0.01]);
    }
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
    // dovetail grooves: open at the bottom end, closed 4 mm before the top
    for (gx = groove_xs) translate([gx, out_y0 - 1, back_z - 0.01]) prism_y(out_y1 - out_y0 - 4 + 1) dovetail2d(groove_mouth, groove_d);
    // latch pocket on the back of the latch block
    translate([latch_x - 5.5, pocket_y0, back_z - 0.01]) cube([11, pocket_len, 1.4]);
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
  echo("FIT", "board_z", board_z); echo("FIT", "lid_inner_z", lid_inner_z); echo("FIT", "groove_w", groove_mouth);
  echo("FIT", "bosses", [for (h = holes) [h[0], h[1]]]);
  echo("FIT", "usb_cut", [usb_y - usb_plug_w / 2, usb_y + usb_plug_w / 2, usb_cz - usb_plug_h / 2, usb_cz + usb_plug_h / 2]);
  for (c = conn) echo("FIT", str("conn_cut_", c[0]), conn_cut(c));
}

module bracket() {
  plate_t = 3; tongue_t = 1.6; air = plate_t - tongue_t;
  py0 = out_y0 + 5; py1 = out_y1;                 // plate (the release tab sticks out above it)
  rail_w = groove_mouth - 2 * dovetail; rail_h = groove_d - 0.3;
  rail_y1 = out_y1 - 4;                           // rail top meets the groove's closed end: sets the hanging height
  tz = back_z - plate_t;                          // plate back face (against the wall)
  t_y0 = out_y1 - 30;                             // tongue root
  difference() {
    union() {
      rbox(out_x0, py0, out_x1, py1, tz, back_z);
      for (gx = groove_xs) translate([gx, py0, back_z - 0.01]) prism_y(rail_y1 - py0) dovetail2d(rail_w, rail_h);
    }
    // wall screws (4 mm, countersunk from the front)
    for (y = [py0 + 15, t_y0 - 12]) translate([latch_x, y, tz - 0.01]) {
      cylinder(d = 4.5, h = plate_t + 1);
      translate([0, 0, plate_t - 2.2]) cylinder(d1 = 4.5, d2 = 9, h = 2.21);
    }
    // latch tongue: 1 mm slots on both sides, open at the top; 1.4 mm of air behind it
    for (s = [-1, 1]) translate([latch_x + s * 5.5 - 0.5, t_y0, tz - 0.01]) cube([1, py1 - t_y0 + 0.01, plate_t + 0.02]);
    translate([latch_x - 5, t_y0 + 1, tz - 0.01]) cube([10, py1 - t_y0, air + 0.01]);
  }
  // release tab: continues the tongue 10 mm above the box
  translate([latch_x - 5, py1 - 0.01, tz + air]) cube([10, 10, tongue_t]);
  // catch: vertical face at the bottom (stops the box lifting), ramp on top (lets the box slide down)
  translate([latch_x - 5, pocket_y0 + 0.2, back_z - 0.01]) rotate([90, 0, 90]) linear_extrude(10)
    polygon([[0, 0], [0, 1.2], [1.2, 1.2], [3.2, 0]]);
  echo("FIT", "rail_w", rail_w);
}

if (part == "base") base();
else if (part == "bracket") bracket();
else if (part == "assembly") { color("gray") bracket(); color("white", 0.6) base(); color("green") cube([pcb_w, pcb_h, pcb_t]); }
