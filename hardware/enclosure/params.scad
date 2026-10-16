// params.scad — print tolerances and wall sizes. Tune after printing the fittest part.
slip = 0.3;        // clearance for parts that slide or insert by hand
dovetail = 0.2;    // per-side clearance of the bracket rails in the base grooves
wall = 2.4;        // side walls
floor_t = 2.0;     // base floor
lid_t = 2.0;       // lid plate
standoff = 5.0;    // board underside above the floor
insert_d = 4.0;    // M3 heat-set insert hole (M3 x 5 x 4 mm inserts)
insert_h = 5.5;    // hole depth (insert length + 0.5)
corner_r = 3.0;    // outer corner radius
$fn = 48;
