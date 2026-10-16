// params.scad — print tolerances and wall sizes. Tune after printing the fittest part.
slip = 0.3;        // clearance for parts that slide or insert by hand
wall = 2.4;        // side walls
floor_t = 2.0;     // base floor
lid_t = 2.0;       // lid plate
standoff = 5.0;    // board underside above the floor
insert_d = 4.0;    // M3 heat-set insert hole (M3 x 5 x 4 mm inserts)
insert_h = 5.5;    // hole depth (insert length + 0.5)
corner_r = 3.0;    // outer corner radius
$fn = 48;
lid_screw_len = 25;  // M3 countersunk lid screws; the lid counterbores the heads so the tips land 4 mm into the inserts
wall_screw_d = 4.0;       // the box hangs on two pan-head (or round-head) wood screws in its keyholes:
wall_screw_head_d = 8.0;  //   shank, head diameter and head height. Countersunk heads don't work in keyholes.
wall_screw_head_h = 3.2;
