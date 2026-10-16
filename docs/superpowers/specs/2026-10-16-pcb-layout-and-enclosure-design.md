# PCB layout and 3D-printed enclosure (rev 1) — design

Date: 2026-10-16
Builds on: `2026-10-16-carrier-pcb-schematic-design.md` (the rev 1 carrier schematic, PR #1)

## Goal

1. Turn the rev 1 carrier schematic into a routed 2-layer PCB, ready to order from **Aisler** as a
   bare board (hand-soldered).
2. Design a new wall-mounted enclosure, printed in **PLA** on a **Bambu Lab P1S**. The old STLs in
   `docs/` stay as they are, for the prototype.

## User requirements (verbatim intent)

- The enclosure is wall mountable.
- The LEDs are visible with the case closed.
- The power, relay and reed connectors are reachable, as **pluggable** connectors: cables can be
  unplugged without opening the case.
- With the case closed, the devkit's **BOOT** button (GPIO0, Wi-Fi/factory reset), its **EN**
  button and its **USB** port are reachable.
- Antenna: a **SMA bulkhead** on the case wall, fed by a U.FL→SMA pigtail from the WROOM-32U.
- Mounting: a separate **wall bracket**; the box slides onto it and latches.
  *(As built: two **keyholes** in the back instead. The dovetails and latch needed supports that
  couldn't be removed; the box now hangs on two pan-head wall screws.)*
- Firmware stays untouched.

## Constraints

- Fab: Aisler 2-layer, 1.6 mm FR4, 35 µm copper, HASL. The design rules sit above Aisler's minimums
  to leave margin: track ≥0.25 mm, clearance ≥0.2 mm, drill ≥0.3 mm, vias 0.6/0.3 mm, annular
  ring ≥0.2 mm. Aisler compensates plated holes by +0.15 mm, so footprint pad sizes are kept at
  KiCad defaults or larger.
- Hand soldering: the existing 0805 / SOT-23 / SOD-323 / SMA / 1812 footprints are kept, with no
  parts on the bottom side.
- PLA softens at about 55 °C. Don't mount the box in direct sun or on the opener motor; if needed,
  the same files print in PETG.
- Tools: KiCad 10.0.6 (`kicad-cli` and the bundled `pcbnew` Python), Freerouting (a Java jar,
  run with the installed OpenJDK 21), and OpenSCAD (`brew install --cask openscad`).

## Part 1 — Schematic changes (footprints only, nets unchanged)

| Ref | Was | Becomes | Mating plug (BOM "also needed") |
|-----|-----|---------|------|
| J1 (5 V in) | 2-pin screw terminal | `Connector_Phoenix_MC:PhoenixContact_MC_1,5_2-G-3.81_1x02_P3.81mm_Horizontal` | Phoenix MC 1,5/2-ST-3,81 |
| J2 (relay COM/NO/NC) | 3-pin screw terminal | `Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_3-G-5,08_1x03_P5.08mm_Horizontal` | Phoenix MSTB 2,5/3-ST-5,08 |
| J3 (reed) | 2-pin screw terminal | `Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal` | Phoenix MSTB 2,5/2-ST-5,08 |

The power connector has a different pitch (3.81 mm) on purpose. J1 and J3 are both 2-pin, so a
5.08 mm power plug would fit the reed socket and put 5 V on GPIO4; with a 3.81 mm power plug it
physically can't. Generic "2EDG"/"15EDG" clones of these plugs fit and are fine.

`check_schematic.py`, `bom.csv`, the BOM PDF, the schematic PDF and the README are updated to match.
The checker's net list does not change.

## Part 2 — PCB layout

### Orientation (as mounted on the wall, looking at the lid)

```
 ┌───────────────────────────────────────────────┐
 │ H1                                         H2 │
 │   ┌───────────────────────────────────────┐   │
 │USB│        ESP32 DevKitC (U1, sockets)    │U.FL│ ← SMA on right wall
 │◄──│ EN BOOT                               │   │
 │   └───────────────────────────────────────┘   │
 │   D4 RUN  D5 DOOR     K1 relay       F1 D1   │
 │   (LEDs under lid holes)   Q1 R1 R2 D2  C1 C2  │
 │                                     ┌───────┐ │
 │ H3                                  │ U2    │H4│
 │ [J1 3.81] [J3 reed] [J2 COM NO NC]  │ DHT22 │ │
 └──┬──────────┬──────────┬────────────┴───────┴─┘
  power      reed       relay   ← plugs enter from below (bottom wall)
```

- **U1:** the devkit lies horizontally in the upper half. The USB end is at the left board edge;
  the module end with the U.FL socket is on the right.
- **J1, J3, J2:** along the bottom edge, header openings facing down (outwards), connector faces
  flush with the board edge.
- **U2 (DHT22):** in the bottom-right corner, as far as possible from the relay coil, D1/F1 and
  the ESP32, next to the case vents.
- **D4 and D5:** in the free strip between the devkit and the bottom section, at fixed coordinates
  that the enclosure reads.
- **K1, Q1 and the flyback parts:** grouped, with K1 close to J2.
- **H1–H4:** M3 holes at the corners, inset 4 mm.
- **Board size:** about 95 × 70 mm, final value set at layout. The board edge, H1–H4, J1–J3, D4/D5,
  U1 and U2 are **locked**; everything else may move.

### Routing

- Layout is done by a script (`layout.py`) that uses the `pcbnew` Python API to set the outline,
  the design rules and the placement.
- Freerouting routes the signals. Power nets are drawn by the script first: +5V, the relay coil
  path and GND returns are 0.8 mm; +3V3 is 0.5 mm; signals are 0.25 mm.
- Both layers get a GND fill. The bottom fill is solid; on the top it fills free space.
- K1's contact nets (COM/NO/NC → J2) are kept ≥2 mm from all other copper, with no fill near them.
  The opener's button circuit is low voltage, but the margin costs nothing.
- Nothing is placed under the devkit's U.FL/module end, so the pigtail has room.
- Silkscreen: J1 "5V IN +/−", J2 "COM NO NC", J3 "REED", the LED names, "rev 1" and a polarity
  mark on J1.

### Checks

- `kicad-cli pcb drc` reports 0 errors and 0 unconnected items, against the rules above.
- `check_schematic.py` still prints SCHEMATIC OK, and ERC reports 0 errors.
- The PCB and the schematic match: DRC "schematic parity" is enabled and reports no issues.
- A new `check_pcb.py` checks that:
  - every connector opening points at the bottom edge and lies within 0.5 mm of it;
  - U1's USB end points at the left edge;
  - the H1–H4 positions match `board_dims.scad`;
  - the board measures under 100 × 75 mm.
- Fab output in `hardware/garage-opener-pcb/fab/`: Gerbers, drill file and a zip, plus
  `garage-opener-pcb.step` (with 3D models) for the enclosure fit check.

## Part 3 — Enclosure (`hardware/enclosure/`)

### Source

- `enclosure.scad`: one parametric file. A `part` variable selects
  `base | lid | btn_boot | btn_en | lightpipe | fittest | template | assembly`.
- `board_dims.scad`: **generated** by `export_board_dims.py` from the `.kicad_pcb`. It holds the
  board size, hole positions, connector positions and widths, LED and DHT positions, and the U1
  position.
- `devkit_dims.scad`: hand-written values for the devkit (USB position and size, BOOT/EN button
  positions, U.FL position, the height above the carrier). These are **parameters to measure** on
  the real devkit; defaults come from Espressif's DevKitC V4 drawing.
- `params.scad`: print tolerances. Defaults: 0.3 mm slip fit, 2.4 mm walls,
  2.0 mm floor and lid, and the wall screw (Ø4 shank, Ø8 × 3.2 mm pan head).

### Parts

1. **Wall mounting.** *(As built: no bracket.)* Two keyholes in the back of the base, 64 mm apart
   near the top. Each is a Ø9 hole the screw head passes through, a 4.5 mm neck slot next to the
   wall running 10 mm up from it, and a Ø9 head channel behind the 2 mm neck, open to the cavity.
   Printed back-down, the channel only steps outwards, so nothing overhangs. A flat drilling
   template (`template`) has the box outline and a hole where each screw ends up.
2. **Base.**
   - A tray, with outside size about board + 2×(wall + 1 mm) by height ≈ 38 mm. The height comes
     from the relay (15.5 mm) or the devkit on its sockets (≈ 8.5 socket + 2.5 male-header strip
     + 1.6 + module), plus 5 mm standoffs.
   - A solid 4 mm back slab (the base prints lying on it), with the two keyholes cut into it and
     the floor.
   - Four bosses with M3 heat-set insert holes (Ø4.0 mm, 5 mm deep) under H1–H4.
   - **Bottom wall:** a cut-out per connector, sized for the mating plug: the header width plus
     1.5 mm each side, and its height plus 3 mm.
   - **Left wall:** a USB cut-out, sized to the devkit's USB connector plus room for a cable
     overmold (12 × 8 mm).
   - **Right wall:** a D-shaped hole for the SMA bulkhead (6.5 mm with a 5.8 mm flat), so the
     bulkhead can't twist.
   - **DHT corner:** vent slots in the bottom and right walls, plus an internal baffle wall that
     separates the corner from the relay and ESP32 area.
   - No separate lid pillars. The lid screws go through tubes on the lid, through H1–H4, into the
     boss inserts, clamping the PCB. *(As built: a side strip for pillars would have put the USB
     socket out of a plug's reach.)*
3. **Lid.**
   - Four countersunk, counterbored M3 holes over the clamping tubes (the screw tip lands 4 mm into the insert).
   - Two Ø3.2 mm LED holes, each with a printed tube that reaches down to 1 mm above D4/D5. The
     tubes stop light bleeding between the LEDs.
   - Two button sleeves above BOOT and EN. EN's sleeve is recessed 1.5 mm so it can't be pressed
     by accident.
   - Vent slots over the DHT corner.
   - Debossed labels: RUN, DOOR, BOOT, RST, plus "Garage" on the top.
4. **Buttons (btn_boot, btn_en).** Separate push-rods with a flange that keeps them captive in
   their sleeve. They reach down to the devkit's tactile switches, with 0.5 mm of clearance at rest.
   The cap stands 1.5 mm proud of the lid so a press can travel through the gap and the switch.
5. **Light pipes (optional).** Clear or white PLA plugs for the LED holes.
6. **Fit-test piece.** The bottom 16 mm of the base (all three plug cut-outs) and a coupon of the
   back around one keyhole, to check the plugs and the screw before printing the full box.

### Print notes (README)

- PLA, 0.2 mm layers, 4 walls, 20 % gyroid infill, no supports.
- Orientation: base lying on its back slab; lid outside face down, so the labels sit on the textured
  plate; template flat; buttons standing upright.

### Hardware to buy (README and BOM PDF "also needed")

- U.FL (IPEX1) → SMA-female bulkhead pigtail, about 10 cm, and a 2.4 GHz SMA antenna.
- 4 × M3 heat-set inserts (M3 × 5 × 4 mm) and 4 × M3×25 countersunk screws (lid + PCB clamp).
- 2 × 4 mm pan-head wall screws (head ≤ Ø8 × 3.2 mm) and wall plugs.
- Mating plugs: MC 1,5/2-ST-3,81 ×1, MSTB 2,5/2-ST-5,08 ×1, MSTB 2,5/3-ST-5,08 ×1.

### Checks

- `make_stl.sh` renders every part with `openscad -o` without errors and writes STLs to
  `hardware/enclosure/stl/` and PNG previews to `hardware/enclosure/renders/`.
- `check_fit.py` checks, numerically, that:
  - the PCB board-edge box (plus the U1 and K1 heights) fits inside the base cavity, with the
    board at its standoff height;
  - each connector cut-out spans its connector's width;
  - each LED hole lies within 0.5 mm of its LED in X/Y;
  - each button rod lies within 0.5 mm of its switch;
  - the lid's inner surface clears the tallest part by ≥1 mm;
  - nothing in the back of the base or the fit-test piece faces down (no supports), and the
    keyholes fit the wall screw and match the drilling template.
- An `assembly` render shows the PCB's STEP outline in place, for visual review.

## Bring-up / acceptance

1. Measure the devkit (row spacing 25.4 mm, USB position, button positions) and update
   `devkit_dims.scad`.
2. Print the fit-test piece and adjust `params.scad` if needed.
3. Order the PCB from Aisler. Check `fab/garage-opener-pcb.zip` in Aisler's online viewer first.
4. After assembly, run the bring-up checklist from the schematic spec (supply ≥5.1 V; 3V3 ≥3.2 V
   during Wi-Fi; the relay COM–NO check).
5. Print the box, fit it, and check that the LEDs are visible, both buttons work, USB plugs in, and
   all three plugs seat.

## Out of scope

- Firmware changes, including reading the board-ID straps.
- Ordering, and JLC/PCBWay assembly files.
- IP-rated sealing.
