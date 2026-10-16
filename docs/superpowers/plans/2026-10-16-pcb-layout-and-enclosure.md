# PCB Layout and Enclosure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Route the rev 1 carrier PCB for Aisler (2-layer, bare board) and design a PLA wall-mounted enclosure (bracket + slide-on base + lid) whose cut-outs are generated from the PCB.

**Architecture:**
- A script (`tools/layout.py`, bundled KiCad Python + `pcbnew`) builds `garage-opener-pcb.kicad_pcb`
  from the schematic netlist: outline, net classes, locked placement and a GND fill.
- `tools/route.py` exports Specctra DSN, runs Freerouting headless, imports the SES and refills
  the zones.
- `check_pcb.py` gates placement, DRC and parity.
- `export_board_dims.py` turns the routed board into `hardware/enclosure/board_dims.scad`.
- `enclosure.scad` builds every printed part from it.
- `check_fit.py` verifies the fit numerically, and `make_stl.sh` exports STLs and PNG renders.

**Tech Stack:** KiCad 10.0.6 (`kicad-cli`, bundled Python 3 with `pcbnew`), Freerouting 2.5.0
(Java 21), OpenSCAD 2021.01 (brew cask), Python 3 (system, stdlib only).

**Spec:** `docs/superpowers/specs/2026-10-16-pcb-layout-and-enclosure-design.md`

## Global Constraints

- Firmware untouched: no file under `main/` changes.
- Aisler 2-layer rules with margin:
  - track ≥ 0.25 mm, clearance ≥ 0.2 mm;
  - drill ≥ 0.3 mm, via 0.6/0.3 mm;
  - annular ring ≥ 0.2 mm;
  - copper to board edge ≥ 0.5 mm.
- Track widths and clearances:
  - +5V, GND, the relay coil path (`Net-(D2-A)`) and the fused input (`Net-(J1-Pin_1)`,
    `Net-(D1-A)`): 0.8 mm;
  - +3V3: 0.5 mm;
  - relay contact nets (`Net-(J2-Pin_*)`): 1.0 mm wide, ≥ 2.0 mm clearance to all other copper;
  - everything else: 0.25 mm.
- Board ≤ 100 × 75 mm (target 95 × 70). No parts on the bottom side.
- Locked: board edge, H1–H4, J1–J3, D4/D5, U1, U2 and the DHT22 keep-out.
- Connectors:
  - J1 `Connector_Phoenix_MC:PhoenixContact_MC_1,5_2-G-3.81_1x02_P3.81mm_Horizontal`;
  - J2 `Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_3-G-5,08_1x03_P5.08mm_Horizontal`;
  - J3 `Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal`;
  - all three openings face the bottom board edge, with the connector front flush with it
    (within 0.5 mm).
- U1 (devkit) has its USB end at the left board edge.
- Enclosure:
  - PLA, no supports;
  - tolerances 0.3 mm slip fit, 0.2 mm dovetail;
  - walls 2.4 mm, floor and lid 2.0 mm.
- Commits: no `Co-Authored-By` trailer (user's global CLAUDE.md). Commit on branch
  `hw/carrier-pcb-schematic`; do not push.

## Review Focus

1. The devkit orientation and its pin rotation: a mirrored or wrong rotation would land GPIO nets
   on the wrong socket pins while DRC still passes. `check_pcb.py` checks the actual pad
   coordinates of U1 pins 1, 19, 20 and 38.
2. Connector openings facing inwards (rotated 180°): `check_pcb.py` checks the courtyard's far
   edge is at the board's bottom edge.
3. KiCad (y down) to OpenSCAD (y up) coordinate flip errors, which would mirror every cut-out:
   `test_board_dims.py` checks J1 (left) has a smaller scad X than J2 (right), and U1 (top) a
   larger scad Y than J1 (bottom).
4. Relay contact clearance being silently overridden by Freerouting: DRC runs with a
   `.kicad_dru` rule that enforces 2.0 mm, so a violation fails `check_pcb.py`.
5. The lid colliding with the tallest part (K1 15.3 mm, or the flat-mounted DHT22): `check_fit.py`
   compares the lid's inner Z with the max component heights read from the PCB STL export.

---

## File map

| File | Role |
|------|------|
| `hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch` | J1–J3 footprint fields change |
| `hardware/garage-opener-pcb/check_schematic.py` | + FOOTPRINTS check |
| `hardware/garage-opener-pcb/bom.csv`, `make_bom_pdf.py`, `docs/*.pdf` | regenerated; mating plugs and enclosure hardware added to "also needed" |
| `hardware/garage-opener-pcb/garage-opener-pcb.kicad_pro` | net classes (written by layout.py) |
| `hardware/garage-opener-pcb/garage-opener-pcb.kicad_dru` | custom DRC rules (contact clearance, power widths) |
| `hardware/garage-opener-pcb/tools/layout.py` | builds board from netlist (one-shot; board file is source of truth afterwards) |
| `hardware/garage-opener-pcb/tools/route.py` | DSN → Freerouting → SES → zone fill |
| `hardware/garage-opener-pcb/tools/placement.py` | the placement table shared by layout.py and check_pcb.py |
| `hardware/garage-opener-pcb/garage-opener-pcb.kicad_pcb` | the board |
| `hardware/garage-opener-pcb/check_pcb.py` | placement + DRC + parity gate |
| `hardware/garage-opener-pcb/fab.sh` | Gerbers, drill, zip, STEP, STL |
| `hardware/garage-opener-pcb/fab/` | outputs (committed) |
| `hardware/enclosure/export_board_dims.py` | `.kicad_pcb` + PCB STL → `board_dims.scad` |
| `hardware/enclosure/test_board_dims.py` | coordinate-flip test |
| `hardware/enclosure/{params,devkit_dims,board_dims,enclosure}.scad` | the model |
| `hardware/enclosure/check_fit.py` | numeric fit check |
| `hardware/enclosure/make_stl.sh` | STL + PNG export |
| `hardware/enclosure/{stl,renders}/`, `README.md` | outputs + docs |

Shell variables used throughout (define in each shell):

```bash
REPO=/Users/laek/source/ESP32-homekit-garage-opener
HW=$REPO/hardware/garage-opener-pcb
PYK=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
FR_JAR=$HOME/.local/share/freerouting/freerouting-2.5.0.jar
```

---

### Task 1: Pluggable connector footprints in the schematic

**Files:**
- Modify: `hardware/garage-opener-pcb/check_schematic.py`
- Modify: `hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch`
- Modify: `hardware/garage-opener-pcb/bom.csv`
- Modify: `hardware/garage-opener-pcb/make_bom_pdf.py`
- Modify: `docs/BOM_GarageOpener.pdf`, `docs/Schematic_GarageOpener.pdf`

**Interfaces:**
- Produces: schematic footprints for J1–J3, as in Global Constraints; layout.py (Task 2) reads them
  from the netlist.

- [ ] **Step 1: Add the failing footprint check.** In `check_schematic.py`, after the `VALUES =`
  line, add:

```python
# Pluggable connectors (enclosure design): power has a different pitch so it can't mate with J3.
FOOTPRINTS = {
    "J1": "Connector_Phoenix_MC:PhoenixContact_MC_1,5_2-G-3.81_1x02_P3.81mm_Horizontal",
    "J2": "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_3-G-5,08_1x03_P5.08mm_Horizontal",
    "J3": "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal",
}
```

  Inside the `for comp in root.iter("comp"):` loop, after the VALUES check, add:

```python
    if ref in FOOTPRINTS:
        check(fp == FOOTPRINTS[ref], f"{ref}: footprint {fp!r}, expected {FOOTPRINTS[ref]!r}")
```

- [ ] **Step 2: Run it and watch it fail.**
  - Run: `python3 $HW/check_schematic.py`
  - Expected: exit 1, with three `FAIL: J1/J2/J3: footprint 'TerminalBlock_Phoenix:...'` lines.

- [ ] **Step 3: Change the footprints.** Edit the schematic with a small script that rewrites the
  `Footprint` property inside each J symbol instance:

```bash
python3 - "$HW/garage-opener-pcb.kicad_sch" <<'EOF'
import re, sys
p = sys.argv[1]; s = open(p).read()
NEW = {"J1": "Connector_Phoenix_MC:PhoenixContact_MC_1,5_2-G-3.81_1x02_P3.81mm_Horizontal",
       "J2": "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_3-G-5,08_1x03_P5.08mm_Horizontal",
       "J3": "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal"}
for ref, fp in NEW.items():
    # each placed symbol: (symbol (lib_id ...) ... (property "Reference" "J1" ...) ... (property "Footprint" "..."
    pat = re.compile(r'(\(property "Reference" "%s"(?:(?!\(symbol \(lib_id).)*?\(property "Footprint" ")[^"]*(")' % ref, re.S)
    s, n = pat.subn(lambda m: m.group(1) + fp + m.group(2), s, count=1)
    assert n == 1, ref
open(p, "w").write(s)
EOF
```

- [ ] **Step 4: Run the checker and ERC.**
  - Run: `python3 $HW/check_schematic.py && kicad-cli sch erc --severity-all -o /tmp/claude-501/erc.rpt $HW/garage-opener-pcb.kicad_sch && grep -E 'Total|Errors|errors' /tmp/claude-501/erc.rpt`
  - Expected: `SCHEMATIC OK`; ERC reports 0 errors and the 1 known MMBT2222A warning.

- [ ] **Step 5: Regenerate the BOM, the BOM PDF and the schematic PDF.** In `make_bom_pdf.py`,
  replace the EXTRA list with:

```python
EXTRA = [
    ("1", "ESP32 DevKitC V4 (ESP32-WROOM-32U)", "Plugs into U1. Needs an external 2.4 GHz antenna on its U.FL connector."),
    ("2", "Female header 1x19, 2.54 mm", "U1 sockets (rows 25.4 mm apart)"),
    ("1", "Plug Phoenix MC 1,5/2-ST-3,81 (or 15EDGK clone)", "Mates with J1 (5 V in)"),
    ("1", "Plug Phoenix MSTB 2,5/2-ST-5,08 (or 2EDGK clone)", "Mates with J3 (reed)"),
    ("1", "Plug Phoenix MSTB 2,5/3-ST-5,08 (or 2EDGK clone)", "Mates with J2 (relay COM/NO/NC)"),
    ("1", "U.FL (IPEX1) to SMA-female bulkhead pigtail, ~10 cm + 2.4 GHz SMA antenna", "Antenna through the enclosure wall"),
    ("8", "M3 heat-set insert, M3×5×4 mm", "Enclosure: 4 for the PCB, 4 for the lid"),
    ("4", "M3×8 screw", "PCB to base (H1–H4)"),
    ("4", "M3×10 countersunk screw", "Lid"),
    ("2", "4 mm wood screw + wall plug", "Wall bracket"),
    ("1", "5.1–5.2 V DC power supply", "Into J1. A plain 5.0 V supply leaves too little margin."),
]
```

  The U2 socket line is removed on purpose: the DHT22 is soldered flat (see Task 2's ruling).
  Then run:

```bash
cd $HW && kicad-cli sch export bom -o bom.csv --fields 'Reference,Value,Footprint,LCSC,${QUANTITY}' \
  --labels 'Refs,Value,Footprint,LCSC,Qty' --group-by 'Value,Footprint,LCSC' --exclude-dnp garage-opener-pcb.kicad_sch \
  && grep -c Phoenix bom.csv && python3 make_bom_pdf.py \
  && kicad-cli sch export pdf -o $REPO/docs/Schematic_GarageOpener.pdf garage-opener-pcb.kicad_sch; cd $REPO
```

  - Expected: `grep -c` prints 2 or 3 (J1 has its own line; J2 and J3 have their own lines), then
    `wrote docs/BOM_GarageOpener.pdf (26 fitted parts)`.

- [ ] **Step 6: Update the README.** In `hardware/garage-opener-pcb/README.md`, replace the line
  `- Relay contacts follow KiCad's SRD symbol (J2: 1 COM, 2 NO, 3 NC)...` block's preceding bullets
  with an additional bullet:
  `- J1–J3 are pluggable Phoenix-style headers. J1 (power) is 3.81 mm pitch and J2/J3 are 5.08 mm, so the power plug can't go into the reed socket.`

- [ ] **Step 7: Commit.**

```bash
cd $REPO && git add hardware/garage-opener-pcb docs/BOM_GarageOpener.pdf docs/Schematic_GarageOpener.pdf \
  && git reset -q hardware/garage-opener-pcb/garage-opener-pcb.kicad_prl \
  && git commit -m "feat(hw): pluggable Phoenix connectors for J1-J3 (3.81 mm power, 5.08 mm relay/reed)"
```

---

### Task 2: Board outline, net classes and locked placement

**Files:**
- Create: `hardware/garage-opener-pcb/tools/placement.py`
- Create: `hardware/garage-opener-pcb/tools/layout.py`
- Create: `hardware/garage-opener-pcb/check_pcb.py`
- Create: `hardware/garage-opener-pcb/garage-opener-pcb.kicad_dru`
- Create: `hardware/garage-opener-pcb/garage-opener-pcb.kicad_pcb`
- Modify: `hardware/garage-opener-pcb/garage-opener-pcb.kicad_pro`
- Modify: `.gitignore` (add `*.kicad_prl`, `fp-info-cache`, `*-backups/`)

**Interfaces:**
- Consumes: the Task 1 footprints, via `kicad-cli sch export netlist`.
- Produces:
  - `placement.py`: `BOARD_W = 95.0`, `BOARD_H = 70.0`, `ORIGIN = (100.0, 100.0)` (page mm of the
    board's top-left corner) and `PLACE: dict[str, tuple[x, y, rot_deg, locked]]` in board mm
    (top-left origin, y down);
  - `DHT_KEEPOUT = (x0, y0, x1, y1)`;
  - `HOLE_INSET = 3.5`.
  - `check_pcb.py` exits 0 when OK. It takes `--placement-only` to skip DRC; Task 3 runs it
    without the flag.

**Ruling for the executor to record:** the DHT22 is soldered flat on the board, with its leads
bent 90° and its grille facing the lid, into U2's existing 1x04 2.54 mm footprint. Standing in a
socket it would be about 34 mm tall and would set the case depth. A keep-out of 15.2 × 25.1 mm
reserves its body area. The footprint stays `PinSocket_1x04_P2.54mm_Vertical`, which is just four
2.54 mm holes, so the schematic doesn't change.

- [ ] **Step 1: Write `tools/placement.py`.** This is the single source of positions. Coordinates
  are worked out from the footprints:
  - The devkit's fab outline runs from y −6.0 (module end) to 48.4 (USB end) relative to pin 1.
    Rotation −90° puts pin 19 (5 V) to the left of pin 1.
  - MSTBA/MC courtyards end 10.54 / 8.54 mm in front of the pins, including a 0.5 mm margin.
  - The relay at −90° has COM at the top and NO/NC at the bottom, towards J2.

```python
"""Rev 1 placement, in board mm: origin = board top-left corner, x right, y down (KiCad view = lid view)."""
BOARD_W, BOARD_H = 95.0, 70.0
ORIGIN = (100.0, 100.0)          # where the board's top-left sits on the KiCad page
HOLE_INSET = 3.5

# U1: pin 1 (3V3) at the module end, pin 19 (5V) at the USB end. The USB end sits 3.18 mm past pin 19,
# so pin 19 at x=3.18 puts the devkit's USB edge on the board edge.
U1_PIN1 = (48.90, 14.00)

def _front(pin_y_front):  # connector pin row y so that the courtyard front (minus 0.5 margin) = board edge
    return BOARD_H - pin_y_front

PLACE = {
    # ref: (x, y, rotation_deg, locked)
    "H1": (HOLE_INSET, HOLE_INSET, 0, True),
    "H2": (BOARD_W - HOLE_INSET, HOLE_INSET, 0, True),
    "H3": (HOLE_INSET, BOARD_H - HOLE_INSET, 0, True),
    "H4": (BOARD_W - HOLE_INSET, BOARD_H - HOLE_INSET, 0, True),
    "U1": (U1_PIN1[0], U1_PIN1[1], -90, True),
    "J1": (22.00, _front(8.04), 0, True),     # MC 3.81, front 8.04 mm from pin row
    "J3": (40.00, _front(10.04), 0, True),    # MSTBA 5.08, front 10.04 mm from pin row
    "J2": (58.00, _front(10.04), 0, True),
    "U2": (84.00, 42.00, -90, True),          # pins run left from pin 1; DHT body lies below the row
    "D4": (6.00, 46.00, 0, True),             # RUN  (pad 1 = K)
    "D5": (12.50, 46.00, 0, True),            # DOOR
    "K1": (66.00, 18.00, -90, False),         # COM top, coil (top corners), NO/NC bottom
    # power, near J1
    "F1": (21.00, 45.00, 0, False),
    "D1": (21.00, 51.00, 0, False),
    "C1": (29.00, 46.00, 0, False),
    "C2": (29.00, 52.00, 0, False),
    # reed, near J3
    "R4": (38.00, 46.00, 0, False),
    "R3": (42.50, 46.00, 0, False),
    "C3": (47.00, 46.00, 0, False),
    "D3": (40.00, 51.50, 0, False),
    # LED resistors
    "R6": (7.00, 52.00, 0, False),
    "R7": (13.50, 52.00, 0, False),
    # relay driver, above the relay coil pins
    "Q1": (60.00, 8.00, 0, False),
    "R1": (55.50, 6.50, 0, False),
    "R2": (55.50, 10.00, 0, False),
    "D2": (66.00, 11.00, 0, False),
    # DHT support
    "R5": (79.00, 38.00, 0, False),
    "C4": (84.00, 38.00, 0, False),
    # board ID straps, above U1 pins 7/8 (x 33.66 / 31.12)
    "R8": (28.00, 7.00, 90, False),
    "R9": (31.00, 7.00, 90, False),
    "R10": (35.00, 7.00, 90, False),
    "R11": (38.00, 7.00, 90, False),
}
# Flat-mounted DHT22 body (15.1 x 25.1 mm + 0.1 margin), centred on U2's pin row, extending downward.
_U2_CX = 84.00 - 1.5 * 2.54
DHT_KEEPOUT = (_U2_CX - 7.6, 43.3, _U2_CX + 7.6, 68.4)
LOCKED = {r for r, v in PLACE.items() if v[3]}
```

- [ ] **Step 2: Write `check_pcb.py` (the failing test).**

```python
#!/usr/bin/env python3
"""Checks garage-opener-pcb.kicad_pcb: locked placement, connector/USB orientation, DRC and parity.

Usage: python3 check_pcb.py [--placement-only]     Requires KiCad 10 (bundled Python, kicad-cli).
"""
import json, os, subprocess, sys, tempfile
from pathlib import Path

PROJ = Path(__file__).resolve().parent
PCB = PROJ / "garage-opener-pcb.kicad_pcb"
PYK = "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3"
sys.path.insert(0, str(PROJ / "tools"))
from placement import BOARD_W, BOARD_H, ORIGIN, PLACE, LOCKED, DHT_KEEPOUT  # noqa: E402

# Geometry is read with KiCad's own Python (pcbnew), which only exists in the bundled interpreter.
PROBE = r'''
import json, sys, pcbnew as p
b = p.LoadBoard(sys.argv[1]); mm = p.ToMM
out = {"edge": None, "fp": {}, "keepouts": []}
bb = b.GetBoardEdgesBoundingBox()
out["edge"] = [mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom())]
for f in b.GetFootprints():
    cy = f.GetCourtyard(p.F_CrtYd).BBox()
    out["fp"][f.GetReference()] = {
        "pos": [mm(f.GetPosition().x), mm(f.GetPosition().y)], "rot": f.GetOrientationDegrees(),
        "locked": f.IsLocked(), "layer": f.GetLayerName(),
        "crt": [mm(cy.GetLeft()), mm(cy.GetTop()), mm(cy.GetRight()), mm(cy.GetBottom())],
        "pads": {pd.GetNumber(): [mm(pd.GetPosition().x), mm(pd.GetPosition().y), pd.GetNetname()] for pd in f.Pads()}}
for z in b.Zones():
    if z.GetIsRuleArea():
        r = z.GetBoundingBox(); out["keepouts"].append([mm(r.GetLeft()), mm(r.GetTop()), mm(r.GetRight()), mm(r.GetBottom())])
print(json.dumps(out))
'''

fails = []
def check(cond, msg):
    if not cond: fails.append(msg)

if not PCB.is_file():
    sys.exit(f"FAIL: {PCB.name} does not exist")
r = subprocess.run([PYK, "-c", PROBE, str(PCB)], capture_output=True, text=True)
if r.returncode != 0:
    sys.exit(f"pcbnew probe failed:\n{r.stderr}")
g = json.loads(r.stdout.strip().splitlines()[-1])
ox, oy = ORIGIN
def rel(x, y): return (x - ox, y - oy)

# Board outline
l, t, rr, bt = g["edge"]
check(abs((rr - l) - BOARD_W) < 0.2 and abs((bt - t) - BOARD_H) < 0.2, f"board is {rr-l:.1f} x {bt-t:.1f}, expected {BOARD_W} x {BOARD_H}")
check(BOARD_W <= 100 and BOARD_H <= 75, "board larger than 100 x 75 mm")

fp = g["fp"]
for ref, (x, y, rot, locked) in PLACE.items():
    f = fp.get(ref)
    check(f is not None, f"{ref} missing from board")
    if not f: continue
    px, py = rel(*f["pos"])
    if locked:
        check(abs(px - x) < 0.01 and abs(py - y) < 0.01, f"{ref} at ({px:.2f},{py:.2f}), locked position is ({x},{y})")
        check(f["locked"], f"{ref} is not locked")
    check(f["layer"] == "F.Cu", f"{ref} is on {f['layer']}, all parts must be on top")
    cl, ct, cr, cb = f["crt"]
    check(cl - ox >= -0.6 and ct - oy >= -0.6 and cr - ox <= BOARD_W + 0.6 and cb - oy <= BOARD_H + 0.6,
          f"{ref} courtyard leaves the board")

# Connector openings face the bottom edge: the courtyard bottom (incl. 0.5 mm margin) is at the edge.
for ref in ("J1", "J2", "J3"):
    f = fp.get(ref)
    if f:
        front = f["crt"][3] - oy - 0.5
        check(abs(front - BOARD_H) <= 0.5, f"{ref} front at y={front:.2f}, expected the bottom edge ({BOARD_H})")
        check(f["crt"][1] - oy < f["pads"]["1"][1] - oy + 0.1, f"{ref} courtyard extends upwards: opening faces inwards")

# Devkit: pin 19 (5V) and pin 20 (GND) are at the USB end, which must be the left edge.
u = fp.get("U1")
if u:
    p1, p19, p20, p38 = (rel(*u["pads"][n][:2]) for n in ("1", "19", "20", "38"))
    check(p19[0] < p1[0] and p20[0] < p38[0], "U1 USB end (pins 19/20) is not on the left")
    check(abs(p19[1] - p1[1]) < 0.01 and abs(p20[1] - p38[1]) < 0.01, "U1 rows are not horizontal")
    check(p1[1] < p38[1], "U1 is mirrored: pin 1 row must be the top row")
    check(u["crt"][0] - ox >= -0.01, "U1 USB end is outside the board")

# DHT keep-out exists where placement.py says.
check(any(all(abs(a - b) < 0.2 for a, b in zip(rel(k[0], k[1]) + rel(k[2], k[3]), DHT_KEEPOUT)) for k in g["keepouts"]),
      "DHT22 keep-out rule area missing or misplaced")

if "--placement-only" not in sys.argv:
    with tempfile.TemporaryDirectory() as tmp:
        rpt = Path(tmp) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--schematic-parity", "--severity-error", "--format", "json",
                        "-o", str(rpt), str(PCB)], capture_output=True, text=True)
        d = json.loads(rpt.read_text())
    errs = [v for v in d.get("violations", []) if v.get("severity") == "error"]
    unc = d.get("unconnected_items", [])
    par = d.get("schematic_parity", [])
    for v in errs[:15]: fails.append(f"DRC {v['type']}: {v['description']} — " + "; ".join(i['description'] for i in v.get('items', [])))
    check(not errs, f"{len(errs)} DRC errors")
    check(not unc, f"{len(unc)} unconnected items")
    check(not par, f"{len(par)} schematic parity issues: " + "; ".join(p.get('description', '') for p in par[:5]))

if fails:
    print("\n".join("FAIL: " + f for f in fails)); sys.exit(1)
print("PCB OK" + (" (placement only)" if "--placement-only" in sys.argv else ""))
```

- [ ] **Step 3: Run it and watch it fail.**
  - Run: `python3 $HW/check_pcb.py --placement-only`
  - Expected: `FAIL: garage-opener-pcb.kicad_pcb does not exist`, exit 1.

- [ ] **Step 4: Write the custom DRC rules** in `garage-opener-pcb.kicad_dru`:

```
(version 1)
(rule "relay contacts: 2 mm from everything else"
  (condition "A.NetName =~ 'Net-(J2-Pin_*' && B.NetName != A.NetName")
  (constraint clearance (min 2.0mm)))
(rule "power track width"
  (condition "A.Type == 'Track' && (A.NetName == '+5V' || A.NetName == 'GND' || A.NetName == 'Net-(D2-A)' || A.NetName == 'Net-(J1-Pin_1)' || A.NetName == 'Net-(D1-A)')")
  (constraint track_width (min 0.8mm)))
(rule "3V3 track width"
  (condition "A.Type == 'Track' && A.NetName == '+3V3'")
  (constraint track_width (min 0.5mm)))
(rule "contact track width"
  (condition "A.Type == 'Track' && A.NetName =~ 'Net-(J2-Pin_*'")
  (constraint track_width (min 1.0mm)))
```

- [ ] **Step 5: Write `tools/layout.py`.** It runs under `$PYK` and does the following:
  - exports the netlist and loads each footprint from KiCad's stock library (or `lib/`);
  - assigns nets, places and locks the parts;
  - draws the board edge and the DHT keep-out;
  - adds the GND zones;
  - writes net classes into the `.kicad_pro`;
  - saves the board.

```python
#!/usr/bin/env python3
"""One-shot: builds garage-opener-pcb.kicad_pcb from the schematic netlist + tools/placement.py.
Run with KiCad's bundled Python. Refuses to overwrite an existing board unless --force (the board is the
source of truth once routed)."""
import json, subprocess, sys, tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import pcbnew as p
sys.path.insert(0, str(Path(__file__).resolve().parent))
from placement import BOARD_W, BOARD_H, ORIGIN, PLACE, DHT_KEEPOUT

PROJ = Path(__file__).resolve().parent.parent
SCH, PCB, PRO = (PROJ / f"garage-opener-pcb.{e}" for e in ("kicad_sch", "kicad_pcb", "kicad_pro"))
STOCK = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints")
if PCB.exists() and "--force" not in sys.argv:
    sys.exit(f"{PCB.name} exists; pass --force to rebuild it from scratch (this discards routing)")
P = lambda x, y: p.VECTOR2I(p.FromMM(ORIGIN[0] + x), p.FromMM(ORIGIN[1] + y))

# 1. Net classes in the project file (read by pcbnew on load and exported to Freerouting's DSN).
pro = json.loads(PRO.read_text())
def nc(name, w, clr): return {"name": name, "track_width": w, "clearance": clr, "via_diameter": 0.6, "via_drill": 0.3,
                              "diff_pair_gap": 0.25, "diff_pair_width": 0.2, "diff_pair_via_gap": 0.25,
                              "microvia_diameter": 0.3, "microvia_drill": 0.1, "wire_width": 6, "bus_width": 12,
                              "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": 0}
pro["net_settings"] = {
    "classes": [nc("Default", 0.25, 0.2), nc("Power", 0.8, 0.2), nc("Supply3V3", 0.5, 0.2), nc("Contact", 1.0, 2.0)],
    "meta": {"version": 4},
    "netclass_patterns": [{"netclass": "Power", "pattern": n} for n in ("+5V", "GND", "Net-(D2-A)", "Net-(J1-Pin_1)", "Net-(D1-A)")]
                       + [{"netclass": "Supply3V3", "pattern": "+3V3"}, {"netclass": "Contact", "pattern": "Net-(J2-Pin_*"}],
}
pro.setdefault("board", {}).setdefault("design_settings", {}).setdefault("rules", {}).update({
    "min_track_width": 0.25, "min_clearance": 0.2, "min_through_hole_diameter": 0.3, "min_via_diameter": 0.6,
    "min_via_annular_width": 0.15, "min_hole_to_hole": 0.5, "min_copper_edge_clearance": 0.5, "min_silk_clearance": 0.0})
PRO.write_text(json.dumps(pro, indent=2))

# 2. Netlist
with tempfile.TemporaryDirectory() as t:
    nx = Path(t) / "n.xml"
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml", "-o", str(nx), str(SCH)], check=True,
                   capture_output=True)
    root = ET.parse(nx).getroot()
comps = {c.get("ref"): (c.findtext("value"), c.findtext("footprint")) for c in root.iter("comp")}
pin_net = {(n.get("ref"), n.get("pin")): net.get("name") for net in root.iter("net") for n in net.iter("node")}
assert set(comps) == set(PLACE), f"placement/netlist mismatch: {set(comps) ^ set(PLACE)}"

board = p.NewBoard(str(PCB))
nets = {}
for name in sorted(set(pin_net.values())):
    ni = p.NETINFO_ITEM(board, name); board.Add(ni); nets[name] = ni

# 3. Footprints
for ref, (val, fpid) in sorted(comps.items()):
    lib, name = fpid.split(":", 1)
    libdir = PROJ / "lib/garage-opener.pretty" if lib == "garage-opener" else STOCK / f"{lib}.pretty"
    f = p.FootprintLoad(str(libdir), name)
    assert f, fpid
    f.SetFPID(p.LIB_ID(lib, name)); f.SetReference(ref); f.SetValue(val)
    x, y, rot, locked = PLACE[ref]
    f.SetPosition(P(x, y)); f.SetOrientationDegrees(rot)
    for pad in f.Pads():
        net = pin_net.get((ref, pad.GetNumber()))
        if net: pad.SetNet(nets[net])
    board.Add(f); f.SetLocked(locked)
    # Parity: link to the schematic symbol by its UUID path
    comp = next(c for c in root.iter("comp") if c.get("ref") == ref)
    tstamps = comp.find("sheetpath").get("tstamps") + comp.findtext("tstamps")
    f.SetPath(p.KIID_PATH(tstamps))

# 4. Board edge (Edge.Cuts rectangle) and DHT keep-out
def seg(a, b, layer):
    s = p.PCB_SHAPE(board); s.SetShape(p.SHAPE_T_SEGMENT); s.SetStart(a); s.SetEnd(b); s.SetLayer(layer); s.SetWidth(p.FromMM(0.1)); board.Add(s)
c = [P(0, 0), P(BOARD_W, 0), P(BOARD_W, BOARD_H), P(0, BOARD_H)]
for i in range(4): seg(c[i], c[(i + 1) % 4], p.Edge_Cuts)

def zone(rect, layerset, rule_area, net=None, prio=0):
    z = p.ZONE(board); z.SetLayerSet(layerset)
    x0, y0, x1, y1 = rect
    ol = z.Outline(); ol.NewOutline()
    for (x, y) in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)): ol.Append(P(x, y))
    if rule_area:
        z.SetIsRuleArea(True); z.SetDoNotAllowFootprints(True); z.SetDoNotAllowTracks(False)
        z.SetDoNotAllowVias(False); z.SetDoNotAllowPads(False); z.SetDoNotAllowZoneFills(False); z.SetZoneName("DHT22 body")
    else:
        z.SetNet(net); z.SetAssignedPriority(prio); z.SetLocalClearance(p.FromMM(0.3)); z.SetMinThickness(p.FromMM(0.25))
        z.SetPadConnection(p.ZONE_CONNECTION_THERMAL); z.SetThermalReliefGap(p.FromMM(0.4)); z.SetThermalReliefSpokeWidth(p.FromMM(0.6))
    board.Add(z); return z
ls = p.LSET(); ls.AddLayer(p.F_Cu)
zone(DHT_KEEPOUT, ls, True)
for layer in (p.F_Cu, p.B_Cu):
    l1 = p.LSET(); l1.AddLayer(layer)
    zone((0.5, 0.5, BOARD_W - 0.5, BOARD_H - 0.5), l1, False, nets["GND"])

board.Save(str(PCB))
print("wrote", PCB.name, len(comps), "footprints")
```

  If any pcbnew call above doesn't exist in 10.0.6, the executor looks it up with
  `$PYK -c "import pcbnew; help(pcbnew.ZONE)"` and records a Ruling. The intent of each call is
  described in its comment.

- [ ] **Step 6: Run layout, then the placement check.**
  - Run: `$PYK $HW/tools/layout.py && python3 $HW/check_pcb.py --placement-only`
  - Expected: `wrote garage-opener-pcb.kicad_pcb 36 footprints`, then `PCB OK (placement only)`.
  - If an orientation check fails (U1 or a J part), fix the rotation in `placement.py`
    (for example −90 → 90) and record a Ruling. Don't change the check.

- [ ] **Step 7: Check courtyards with DRC.**
  - Run: `kicad-cli pcb drc --severity-error --format json -o /tmp/claude-501/drc.json $HW/garage-opener-pcb.kicad_pcb; python3 -c "import json;d=json.load(open('/tmp/claude-501/drc.json'));print(sorted({v['type'] for v in d['violations'] if v['severity']=='error'}))"`
  - Expected: only routing-related types, such as `unconnected_items` and `clearance` inside the
    unfilled zone. There must be no `courtyards_overlap` and no `items_not_allowed` (keep-out).
  - If there is a courtyard overlap, move the **unlocked** part named in the violation by 1–2 mm
    in `placement.py`, rerun Step 6 with `--force`, and repeat.

- [ ] **Step 8: Gitignore and commit.**

```bash
cd $REPO && printf '\n# KiCad local state\n*.kicad_prl\nfp-info-cache\n*-backups/\n' >> .gitignore \
  && git add .gitignore hardware/garage-opener-pcb && git commit -m "feat(hw): PCB outline, net classes and locked placement with checker"
```

---

### Task 3: Routing

**Files:**
- Create: `hardware/garage-opener-pcb/tools/route.py`
- Modify: `hardware/garage-opener-pcb/garage-opener-pcb.kicad_pcb`

**Interfaces:**
- Consumes: the board from Task 2; `check_pcb.py` (full mode).
- Produces: a fully routed board. DRC: 0 errors, 0 unconnected, 0 parity issues.

- [ ] **Step 1: Run the full check and watch it fail.**
  - Run: `python3 $HW/check_pcb.py`
  - Expected: exit 1, with `FAIL: N unconnected items` where N > 0.

- [ ] **Step 2: Install Freerouting.**

```bash
mkdir -p ~/.local/share/freerouting && curl -sL -o $FR_JAR \
  https://github.com/freerouting/freerouting/releases/download/v2.5.0/freerouting-2.5.0.jar && java -jar $FR_JAR --help | head -5
```

  Expected: a usage text. If `--help` isn't recognised, any non-crash output is fine.

- [ ] **Step 3: Write `tools/route.py`.**

```python
#!/usr/bin/env python3
"""Routes garage-opener-pcb.kicad_pcb: exports Specctra DSN, runs Freerouting headless, imports SES, refills zones.
Run with KiCad's bundled Python. Existing locked tracks are kept (exported as protected wires)."""
import os, subprocess, sys
from pathlib import Path
import pcbnew as p

PROJ = Path(__file__).resolve().parent.parent
PCB = PROJ / "garage-opener-pcb.kicad_pcb"
JAR = os.environ.get("FR_JAR", str(Path.home() / ".local/share/freerouting/freerouting-2.5.0.jar"))
WORK = PROJ / "tools/.route"; WORK.mkdir(exist_ok=True)
dsn, ses = WORK / "board.dsn", WORK / "board.ses"

b = p.LoadBoard(str(PCB))
for z in b.Zones():                 # copper pours confuse the autorouter; unfill before export
    if not z.GetIsRuleArea(): z.UnFill()
assert p.ExportSpecctraDSN(b, str(dsn)), "DSN export failed"
r = subprocess.run(["java", "-jar", JAR, "-de", str(dsn), "-do", str(ses), "-mp", "30", "--gui.enabled=false"],
                   capture_output=True, text=True, timeout=900)
print(r.stdout[-2000:], r.stderr[-2000:])
assert ses.is_file(), "Freerouting produced no SES"
assert p.ImportSpecctraSES(b, str(ses)), "SES import failed"
p.ZONE_FILLER(b).Fill(b.Zones())
b.Save(str(PCB))
print("routed", sum(1 for t in b.GetTracks()), "track segments/vias")
```

  Add `tools/.route/` to `.gitignore`.

- [ ] **Step 4: Route, then check.**
  - Run: `$PYK $HW/tools/route.py && python3 $HW/check_pcb.py`
  - Expected: `routed N track segments/vias`, then `PCB OK`.
  - If DRC reports `clearance` on a `Net-(J2-Pin_*)` net or `track_width` errors, Freerouting
    ignored a net class. In that case:
    1. Delete the tracks of the offending nets in a pcbnew script.
    2. Hand-route them in the script as straight segments with 45° corners on `B.Cu`, using the
       pad coordinates printed by check_pcb's probe.
    3. Lock those tracks and rerun Step 4. Record a Ruling.
  - If unconnected items remain, rerun with `-mp 60`. If that still fails, nudge an unlocked part
    that blocks a channel and record it.

- [ ] **Step 5: Look at the board.**
  - Run: `kicad-cli pcb export svg --layers F.Cu,B.Cu,Edge.Cuts,F.Silkscreen --mode-single -o /tmp/claude-501/pcb.svg $HW/garage-opener-pcb.kicad_pcb && qlmanage -t -s 1600 -o /tmp/claude-501 /tmp/claude-501/pcb.svg`
  - Read `/tmp/claude-501/pcb.svg.png`.
  - Expected: no tracks run through the DHT keep-out on the top layer close to the sensor body.
    The GND pours are continuous enough that every GND pad has a thermal connection.

- [ ] **Step 6: Commit.**

```bash
cd $REPO && git add .gitignore hardware/garage-opener-pcb && git commit -m "feat(hw): route rev 1 PCB with Freerouting; DRC, unconnected and parity clean"
```

---

### Task 4: Silkscreen and fab outputs

**Files:**
- Create: `hardware/garage-opener-pcb/tools/silk.py`
- Create: `hardware/garage-opener-pcb/fab.sh`
- Create: `hardware/garage-opener-pcb/fab/` (Gerbers, drill, `garage-opener-pcb-gerbers.zip`, `garage-opener-pcb.step`, `garage-opener-pcb.stl`)
- Modify: `hardware/garage-opener-pcb/README.md`

**Interfaces:**
- Produces:
  - `fab/garage-opener-pcb.stl`: board with components, in KiCad page coordinates (mm, y flipped
    by KiCad's 3D export; Task 5 handles it). Task 5 reads it for component heights.
  - The STEP is for the user only.

- [ ] **Step 1: Write the failing check.** Add to `check_pcb.py`, before the `if fails:` block:

```python
SILK = {"5V IN", "COM NO NC", "REED", "RUN", "DOOR", "rev 1"}
if "--placement-only" not in sys.argv:
    texts = subprocess.run([PYK, "-c", "import sys,pcbnew as p;b=p.LoadBoard(sys.argv[1]);"
                            "print('\\n'.join(d.GetText() for d in b.GetDrawings() if isinstance(d,p.PCB_TEXT)))", str(PCB)],
                           capture_output=True, text=True).stdout.splitlines()
    missing = {s for s in SILK if not any(s in t for t in texts)}
    check(not missing, f"silkscreen labels missing: {sorted(missing)}")
```

  Run `python3 $HW/check_pcb.py`. Expected: `FAIL: silkscreen labels missing: [...]`.

- [ ] **Step 2: Write `tools/silk.py`.** It places 1.0 mm text on F.Silkscreen:
  - "5V IN" with "+" and "−" markers 1.5 mm above J1's pins (pin 1 is "+", F1 side);
  - "COM NO NC" above J2's pins, one word per pin x;
  - "REED" above J3;
  - "RUN" / "DOOR" above D4 / D5;
  - "rev 1" and "garage-opener" near H1.

  It then refills the zones and saves.

```python
#!/usr/bin/env python3
"""Adds silkscreen labels. Run with KiCad's bundled Python; idempotent (removes its own texts first)."""
import sys
from pathlib import Path
import pcbnew as p
sys.path.insert(0, str(Path(__file__).resolve().parent))
from placement import ORIGIN, PLACE

PCB = Path(__file__).resolve().parent.parent / "garage-opener-pcb.kicad_pcb"
b = p.LoadBoard(str(PCB))
for d in list(b.GetDrawings()):            # remove previously generated labels
    if isinstance(d, p.PCB_TEXT) and d.GetLayer() == p.F_SilkS:
        b.Remove(d)
fp = {f.GetReference(): f for f in b.GetFootprints()}
def pad(ref, n):
    v = fp[ref].FindPadByNumber(n).GetPosition(); return p.ToMM(v.x), p.ToMM(v.y)
def text(s, x, y, size=1.0):
    t = p.PCB_TEXT(b); t.SetText(s); t.SetLayer(p.F_SilkS); t.SetPosition(p.VECTOR2I(p.FromMM(x), p.FromMM(y)))
    t.SetTextSize(p.VECTOR2I(p.FromMM(size), p.FromMM(size))); t.SetTextThickness(p.FromMM(0.15)); b.Add(t)
x1, y1 = pad("J1", "1"); x2, _ = pad("J1", "2")
text("5V IN", (x1 + x2) / 2, y1 - 3.0); text("+", x1, y1 - 1.8); text("-", x2, y1 - 1.8)
for n, w in (("1", "COM"), ("2", "NO"), ("3", "NC")):
    x, y = pad("J2", n); text(w, x, y - 2.0)
text("COM NO NC", (pad("J2", "1")[0] + pad("J2", "3")[0]) / 2, pad("J2", "1")[1] - 3.6, 0.8)
x, y = pad("J3", "1"); text("REED", x + 2.54, y - 2.0)
for ref, s in (("D4", "RUN"), ("D5", "DOOR")):
    x, y = pad(ref, "1"); text(s, x + 1.27, y - 2.6)
text("garage-opener  rev 1", ORIGIN[0] + 18, ORIGIN[1] + 3.0, 1.2)
p.ZONE_FILLER(b).Fill(b.Zones())
b.Save(str(PCB)); print("silkscreen labels written")
```

- [ ] **Step 3: Run silk, then check.**
  - Run: `$PYK $HW/tools/silk.py && python3 $HW/check_pcb.py`
  - Expected: `PCB OK`. DRC `silk_overlap` is a warning, not an error, but inspect the SVG
    (Task 3 Step 5 command) and move any label that sits on a pad.

- [ ] **Step 4: Write `fab.sh`.**

```bash
#!/bin/sh
# Writes Aisler-ready Gerbers + drill + zip, and STEP/STL 3D models, into fab/.
set -eu
cd "$(dirname "$0")"
B=garage-opener-pcb.kicad_pcb
rm -rf fab && mkdir -p fab/gerbers
kicad-cli pcb export gerbers -o fab/gerbers/ --layers F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts --subtract-soldermask "$B"
kicad-cli pcb export drill -o fab/gerbers/ --format excellon --excellon-separate-th --generate-map --map-format gerberx2 "$B"
(cd fab/gerbers && zip -q ../garage-opener-pcb-gerbers.zip *)
kicad-cli pcb export step -o fab/garage-opener-pcb.step --subst-models --force "$B"
kicad-cli pcb export stl  -o fab/garage-opener-pcb.stl  --subst-models --force "$B"
ls fab fab/gerbers
```

  - Run: `sh $HW/fab.sh`
  - Expected: `fab/` lists the zip, `.step` and `.stl`. `fab/gerbers` has 9 `.g*` files, 2 drill
    files and a map. If a flag is rejected by kicad-cli 10, check `kicad-cli pcb export gerbers --help`,
    use the closest equivalent and record a Ruling.

- [ ] **Step 5: Update the README** (`hardware/garage-opener-pcb/README.md`). Add a section:

```markdown
## PCB

- `garage-opener-pcb.kicad_pcb` is the source of truth. `tools/layout.py` and `tools/route.py` built rev 1
  from scratch (layout refuses to overwrite without `--force`, which discards routing).
- Check after any edit: `python3 check_pcb.py` (placement, connector/USB orientation, DRC, unconnected, parity).
- Fab files: `sh fab.sh` → upload `fab/garage-opener-pcb-gerbers.zip` to Aisler (2 layers, 1.6 mm, HASL).
  Aisler also accepts the `.kicad_pcb` directly.
- The DHT22 is soldered **flat** (leads bent 90°, grille facing the lid) over the marked keep-out, not in a socket.
```

- [ ] **Step 6: Commit.**

```bash
cd $REPO && git add hardware/garage-opener-pcb && git commit -m "feat(hw): silkscreen labels and Aisler fab outputs (Gerbers, drill, STEP, STL)"
```

---

### Task 5: Board dimensions for the enclosure

**Files:**
- Create: `hardware/enclosure/export_board_dims.py`
- Create: `hardware/enclosure/test_board_dims.py`
- Create: `hardware/enclosure/board_dims.scad` (generated, committed)

**Interfaces:**
- Consumes: `garage-opener-pcb.kicad_pcb`, `fab/garage-opener-pcb.stl`, `tools/placement.py`.
- Produces: `board_dims.scad`. All values are in mm, in **scad coordinates**: origin at the board's
  bottom-left corner on its underside, X right, Y up (towards the top of the wall-mounted box), Z
  out of the board towards the lid. It defines:

```
pcb_w, pcb_h, pcb_t                      // 95, 70, 1.6
holes = [[x,y], ...]                     // H1..H4
conn = [[name, x0, x1, z_top], ...]      // J1, J3, J2 body x-extent at the bottom edge, height above board top
leds = [[name, x, y], ...]               // D4 RUN, D5 DOOR (LED centre)
dht = [x0, y0, x1, y1]                   // keep-out rectangle (flat DHT22 body)
u1_pin1 = [x, y]; u1_pin19 = [x, y]      // devkit socket pins (for devkit_dims.scad offsets)
z_max_by_region = [[name, x0,y0,x1,y1, z_top], ...]   // tallest component height per named region: relay, dht, all
```

- [ ] **Step 1: Write the failing test** in `test_board_dims.py`:

```python
#!/usr/bin/env python3
"""Coordinate-flip and sanity tests for board_dims.scad. Run: python3 test_board_dims.py"""
import re, sys
from pathlib import Path
s = (Path(__file__).parent / "board_dims.scad").read_text()
def num(name): return float(re.search(rf"^{name}\s*=\s*([-\d.]+);", s, re.M).group(1))
def rows(name): return re.findall(r'\["(\w+)",([^\]]*)\]', re.search(rf"^{name}\s*=\s*\[(.*?)\];", s, re.M | re.S).group(1))
fails = []
w, h = num("pcb_w"), num("pcb_h")
if (w, h) != (95.0, 70.0): fails.append(f"board {w}x{h}")
conn = {n: [float(v) for v in vals.split(",") if v.strip()] for n, vals in rows("conn")}
if not conn["J1"][0] < conn["J3"][0] < conn["J2"][0]: fails.append(f"connector order not J1<J3<J2 left→right: {conn}")
leds = {n: [float(v) for v in vals.split(",") if v.strip()] for n, vals in rows("leds")}
if not leds["RUN"][0] < leds["DOOR"][0]: fails.append("RUN must be left of DOOR")
if not all(10 < l[1] < 35 for l in leds.values()): fails.append(f"LEDs should be in the lower half (scad y up): {leds}")
u1 = [float(v) for v in re.search(r"^u1_pin1\s*=\s*\[([^\]]*)\]", s, re.M).group(1).split(",")]
if not u1[1] > 50: fails.append(f"U1 must be near the top (scad y up), got y={u1[1]}")
for n, (x0, x1, z) in conn.items():
    if not (5 < z < 20): fails.append(f"{n} height {z} outside 5..20 mm")
zall = [float(v.split(",")[-1]) for n, v in rows("z_max_by_region") if n == "all"]
if not zall or not (15 <= zall[0] <= 20): fails.append(f"tallest part height suspicious: {zall}")
if fails: print("\n".join("FAIL: " + f for f in fails)); sys.exit(1)
print("BOARD DIMS OK")
```

  - Run: `python3 $REPO/hardware/enclosure/test_board_dims.py`
  - Expected: FileNotFoundError for `board_dims.scad`, which is a failure.

- [ ] **Step 2: Write `export_board_dims.py`.**
  - It reads the footprint geometry through the bundled pcbnew (same probe idea as check_pcb).
  - It reads the STL's triangles with the stdlib (binary STL: 80-byte header, uint32 count, then
    50-byte records).
  - For each connector, it takes the max Z of the STL vertices inside the courtyard XY window.
  - For each region, it takes the max Z inside the rectangle.
  - It writes the scad file.

```python
#!/usr/bin/env python3
"""Generates board_dims.scad from the routed PCB and its STL export (hardware/garage-opener-pcb/fab/).
Run: python3 export_board_dims.py   (requires KiCad; run sh ../garage-opener-pcb/fab.sh first)"""
import json, struct, subprocess, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
PROJ = HERE.parent / "garage-opener-pcb"
sys.path.insert(0, str(PROJ / "tools"))
from placement import BOARD_W, BOARD_H, ORIGIN, DHT_KEEPOUT  # noqa: E402
PYK = "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3"
PCB_T = 1.6
PROBE = r'''
import json, sys, pcbnew as p
b = p.LoadBoard(sys.argv[1]); mm = p.ToMM; o = {}
for f in b.GetFootprints():
    c = f.GetCourtyard(p.F_CrtYd).BBox()
    o[f.GetReference()] = {"pos": [mm(f.GetPosition().x), mm(f.GetPosition().y)],
        "crt": [mm(c.GetLeft()), mm(c.GetTop()), mm(c.GetRight()), mm(c.GetBottom())],
        "pads": {d.GetNumber(): [mm(d.GetPosition().x), mm(d.GetPosition().y)] for d in f.Pads()}}
print(json.dumps(o))'''
g = json.loads(subprocess.run([PYK, "-c", PROBE, str(PROJ / "garage-opener-pcb.kicad_pcb")], capture_output=True,
                              text=True, check=True).stdout.strip().splitlines()[-1])
def sx(x): return round(x - ORIGIN[0], 3)                      # board-relative X
def sy(y): return round(BOARD_H - (y - ORIGIN[1]), 3)          # flip: KiCad y down -> scad y up

# STL vertices. KiCad's 3D export already uses y-up with the board's top face at z=PCB_T and page origin;
# detect the transform from the board's own extent rather than assuming it.
data = (PROJ / "fab/garage-opener-pcb.stl").read_bytes()
if data[:5] == b"solid" and b"facet" in data[:400]:
    sys.exit("ASCII STL not supported; kicad-cli writes binary")
n = struct.unpack_from("<I", data, 80)[0]
V = [struct.unpack_from("<3f", data, 84 + i * 50 + 12 + k * 12) for i in range(n) for k in range(3)]
xs, ys, zs = zip(*V)
# board body spans exactly BOARD_W x BOARD_H in x/y; use the min corner as offset.
x0, y0 = min(xs), min(ys)
assert abs((max(xs) - x0) - BOARD_W) < 2 and abs((max(ys) - y0) - BOARD_H) < 25, "unexpected STL extent"
zb = sorted(zs)[len(zs) // 100]           # ~board underside
def ztop(rect):                             # rect in scad coords
    a, b_, c, d = rect
    zz = [z for x, y, z in V if a <= x - x0 <= c and b_ <= y - y0 <= d]
    return round(max(zz) - zb - PCB_T, 2) if zz else 0.0

def crt_scad(ref, margin=0.5):
    l, t, r, b_ = g[ref]["crt"]
    return [sx(l) + margin, sy(b_) + margin, sx(r) - margin, sy(t) - margin]
conn = []
for ref in ("J1", "J3", "J2"):
    x0_, y0_, x1_, y1_ = crt_scad(ref)
    conn.append((ref, x0_, x1_, ztop((x0_, y0_, x1_, y1_))))
leds = [("RUN", "D4"), ("DOOR", "D5")]
led_xy = [(name, round((sx(g[r]["pads"]["1"][0]) + sx(g[r]["pads"]["2"][0])) / 2, 3), sy(g[r]["pads"]["1"][1])) for name, r in leds]
dk = DHT_KEEPOUT; dht = [round(dk[0], 3), sy(dk[3]), round(dk[2], 3), sy(dk[1])]
regions = [("relay", *crt_scad("K1")), ("dht", *dht), ("all", 0, 0, BOARD_W, BOARD_H)]
L = ["// GENERATED by export_board_dims.py from garage-opener-pcb.kicad_pcb — do not edit.",
     "// scad coords: origin = board bottom-left on its underside, X right, Y up (wall-mounted), Z towards the lid.",
     f"pcb_w = {BOARD_W};", f"pcb_h = {BOARD_H};", f"pcb_t = {PCB_T};",
     "holes = [" + ", ".join(f"[{sx(g[h]['pos'][0])}, {sy(g[h]['pos'][1])}]" for h in ("H1", "H2", "H3", "H4")) + "];",
     "conn = [" + ", ".join(f'["{r}", {a}, {b_}, {z}]' for r, a, b_, z in conn) + "];",
     "leds = [" + ", ".join(f'["{nm}", {x}, {y}]' for nm, x, y in led_xy) + "];",
     f"dht = {dht};",
     f"u1_pin1 = [{sx(g['U1']['pads']['1'][0])}, {sy(g['U1']['pads']['1'][1])}];",
     f"u1_pin19 = [{sx(g['U1']['pads']['19'][0])}, {sy(g['U1']['pads']['19'][1])}];",
     "z_max_by_region = [" + ", ".join(f'["{nm}", {a}, {b_}, {c}, {d}, {ztop((a, b_, c, d))}]' for nm, a, b_, c, d in regions) + "];"]
(HERE / "board_dims.scad").write_text("\n".join(L) + "\n")
print("\n".join(L))
```

  The STL's Y axis: KiCad's 3D export is Y-up, which already matches scad Y-up. Check this from
  the output: J1's `z_top` must be about 8–10 mm (the MC header), not 0. If it is 0, the windows
  are mirrored: change `y - y0` to `(BOARD_H - (y - y0))` and record a Ruling.

- [ ] **Step 3: Run the export, then the test.**
  - Run: `cd $REPO/hardware/enclosure && python3 export_board_dims.py && python3 test_board_dims.py; cd $REPO`
  - Expected: the scad listing, then `BOARD DIMS OK`.

- [ ] **Step 4: Commit.**

```bash
cd $REPO && git add hardware/enclosure && git commit -m "feat(enclosure): generate board_dims.scad from the routed PCB"
```

---

### Task 6: Enclosure base and wall bracket

**Files:**
- Create: `hardware/enclosure/params.scad`
- Create: `hardware/enclosure/devkit_dims.scad`
- Create: `hardware/enclosure/enclosure.scad`
- Create: `hardware/enclosure/make_stl.sh`
- Create: `hardware/enclosure/check_fit.py`

**Interfaces:**
- Consumes: `board_dims.scad`.
- Produces:
  - `enclosure.scad`, which takes `-D 'part="base"'` (or bracket / lid / btn_boot / btn_en /
    lightpipe / fittest / assembly);
  - the derived values, which `check_fit.py` reads by echo: `echo("FIT", name, value)` lines for
    `cav_x0, cav_y0, cav_x1, cav_y1, board_z, lid_inner_z, usb_cut, sma_center, conn_cut_<J>`;
  - `make_stl.sh`, which writes `stl/<part>.stl` and `renders/<part>.png`.

- [ ] **Step 1: Install OpenSCAD.**
  - Run: `brew install --cask openscad && /Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD --version`
  - Expected: `OpenSCAD version 2021.01`.
  - Set `OSCAD=/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD`.

- [ ] **Step 2: Write the failing `check_fit.py`.**

```python
#!/usr/bin/env python3
"""Numeric fit check: runs OpenSCAD on enclosure.scad, collects echo("FIT", ...) values, and compares them with
board_dims.scad and devkit_dims.scad. Run: python3 check_fit.py   Exit 0 = OK."""
import re, subprocess, sys, tempfile
from pathlib import Path
HERE = Path(__file__).resolve().parent
OSCAD = "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"
def fit(part):
    with tempfile.TemporaryDirectory() as t:
        r = subprocess.run([OSCAD, "-o", f"{t}/x.echo", "-D", f'part="{part}"', str(HERE / "enclosure.scad")],
                           capture_output=True, text=True)
        txt = Path(f"{t}/x.echo").read_text() if Path(f"{t}/x.echo").exists() else ""
    if r.returncode != 0: sys.exit(f"openscad failed for {part}:\n{r.stderr[-1500:]}")
    out = {}
    for m in re.finditer(r'ECHO: "FIT", "(\w+)", (.+)', txt):
        out[m.group(1)] = eval(m.group(2).replace("true", "True").replace("false", "False"))
    return out
def scad_vals(name):
    s = (HERE / name).read_text(); env = {}
    for m in re.finditer(r"^(\w+)\s*=\s*(.+?);", s, re.M):
        try: env[m.group(1)] = eval(m.group(2).replace("true", "True").replace("false", "False"), {}, env)
        except Exception: pass
    return env
B, D, PR = scad_vals("board_dims.scad"), scad_vals("devkit_dims.scad"), scad_vals("params.scad")
fails = []
def check(c, msg):
    if not c: fails.append(msg)

f = fit("base")
check(f, "base emitted no FIT values")
if f:
    gap = PR["slip"]
    check(f["cav_x0"] <= -gap + 1e-6 and f["cav_x1"] >= B["pcb_w"] + gap - 1e-6, f"cavity x {f['cav_x0']}..{f['cav_x1']} doesn't clear the board")
    check(f["cav_y0"] <= -gap + 1e-6 and f["cav_y1"] >= B["pcb_h"] + gap - 1e-6, f"cavity y {f['cav_y0']}..{f['cav_y1']} doesn't clear the board")
    for name, x0, x1, z in B["conn"]:
        c = f.get(f"conn_cut_{name}")
        check(c and c[0] <= x0 - gap + 1e-6 and c[1] >= x1 + gap - 1e-6 and c[3] >= f["board_z"] + B["pcb_t"] + z,
              f"{name} cut-out {c} doesn't span the connector {x0}..{x1}, top z {z}")
    u = f.get("usb_cut")   # [y0, y1, z0, z1]
    uy = B["u1_pin19"][1] + D["usb_dy"]; uz = f["board_z"] + B["pcb_t"] + D["usb_z"]
    check(u and u[0] <= uy - D["usb_w"] / 2 and u[1] >= uy + D["usb_w"] / 2 and u[2] <= uz - D["usb_hgt"] / 2 and u[3] >= uz + D["usb_hgt"] / 2,
          f"USB cut-out {u} doesn't cover the devkit USB (y {uy}, z {uz})")
    tall = max(r[-1] for r in B["z_max_by_region"])
    check(f["lid_inner_z"] >= f["board_z"] + B["pcb_t"] + max(tall, D["stack_h"]) + 1.0,
          f"lid inner z {f['lid_inner_z']} < tallest part {max(tall, D['stack_h'])} + 1 mm")

g = fit("bracket")
check(g.get("rail_w") and f and abs(g["rail_w"] - f["groove_w"] + 2 * PR["dovetail"]) < 1e-6,
      f"bracket rail {g.get('rail_w')} vs base groove {f.get('groove_w') if f else None}: clearance must be {PR['dovetail']} per side")

if fails: print("\n".join("FAIL: " + x for x in fails)); sys.exit(1)
print("FIT OK")
```

  - Run: `python3 $REPO/hardware/enclosure/check_fit.py`
  - Expected: it fails, either because OpenSCAD is missing the file or with a KeyError on
    `devkit_dims.scad`.

- [ ] **Step 3: Write `params.scad` and `devkit_dims.scad`.**

```openscad
// params.scad — print tolerances and wall sizes. Tune after printing the fittest part.
slip = 0.3;        // clearance for parts that slide or insert by hand
dovetail = 0.2;    // per-side clearance of the bracket rails in the base grooves
wall = 2.4;        // side walls
floor_t = 2.0;     // base floor
lid_t = 2.0;       // lid plate
standoff = 5.0;    // board underside above the floor
insert_d = 4.0;    // M3 heat-set insert hole
insert_h = 5.0;
corner_r = 3.0;    // outer corner radius
$fn = 48;
```

```openscad
// devkit_dims.scad — the ESP32 DevKitC V4 clone on its sockets. MEASURE YOUR DEVKIT and correct these.
// Offsets are relative to socket pin 19 (5V, USB end) in scad coords (X right, Y up), heights above the carrier top.
socket_h = 8.5;          // female header height
devkit_t = 1.6;          // devkit PCB thickness
stack_h = socket_h + devkit_t + 3.5;   // top of the tallest devkit part (module / buttons)
usb_dy = -12.7;          // USB connector centre, Y offset from pin 19 (it sits midway between the rows)
usb_w = 8.0;             // receptacle width (micro-USB 7.5, USB-C 8.9 — set to yours)
usb_hgt = 3.2;           // receptacle height
usb_z = socket_h + devkit_t + usb_hgt / 2;   // receptacle centre height
usb_plug_w = 12;         // cut-out for the cable overmold
usb_plug_h = 8;
btn_boot = [2.0, -20.5]; // BOOT switch centre, offset from pin 19 (X into the board, Y)
btn_en   = [2.0, -4.9];  // EN switch centre
btn_top_z = socket_h + devkit_t + 2.0;   // top of the tactile switch plungers
ufl = [51.0, -2.5];      // U.FL socket on the WROOM-32U, offset from pin 19
```

- [ ] **Step 4: Write `enclosure.scad` (base and bracket; the lid and small parts come in
  Task 7).** The geometry:
  - **Base:** a rounded box whose cavity clears the board by `slip` on each side. Its floor is at
    z = −standoff − floor_t, and the board underside is at z = 0.
    - Walls rise to `lid_inner_z`. The lid sits on top; `lid_inner_z` = max part height + 1.5 mm.
    - Four bosses under H1–H4, from the floor to z = 0, with insert holes.
    - Four corner pillars for the lid inserts, outside the board outline. The cavity is enlarged
      by 7 mm in X on both sides to make room for them.
    - **Bottom wall cut-outs** for J1/J3/J2: X span = connector body + slip; Z from the board top
      to z_top + slip.
    - **Left wall USB cut-out** (usb_plug_w × usb_plug_h, centred on the USB).
    - **Right wall SMA D-hole** (Ø6.5 mm with a 5.8 mm flat), at mid cavity height, at Y = U1
      pin 1 row + 4 mm.
    - **DHT vents:** 2 × 12 mm slots, 1.5 mm pitch, in the bottom wall under the DHT region and
      in the right wall beside it.
    - **DHT baffle:** a 1.2 mm wall from the floor to the board underside along the keep-out's top
      and left edges. On the lid it is mirrored as a wall down to 1 mm above the parts (Task 7).
    - **Back** (outside of the floor): two dovetail-groove housings, 14 mm wide and 4 mm deep,
      running along Y. The groove is 10 mm at its mouth, widening at 45° and 3.2 mm deep, open at
      the bottom end and closed 4 mm before the top end.
    - **Latch block:** 14 × 10 × 4 mm at the top centre, with a 1.2 mm deep pocket facing the wall.
  - **Bracket:** a 3 mm plate, the size of the box's back footprint minus 10 mm in Y.
    - Two dovetail rails that match the grooves minus `dovetail` per side; each rail is 3 mm high.
    - Two countersunk Ø4.5 mm holes on the centre line.
    - A latch tongue cut free on three sides (1.0 mm slots), 1.6 mm thick at the front face, with
      1.4 mm of air behind it. It has a 1.2 mm ramp bump that lines up with the latch pocket when
      the box is fully down, and a 10 mm tab that sticks out above the box top for release.

```openscad
// enclosure.scad — garage-opener rev 1 enclosure. part = "base" | "bracket" | "lid" | "btn_boot" | "btn_en"
//                  | "lightpipe" | "fittest" | "assembly". All dimensions mm; coordinates as in board_dims.scad.
include <params.scad>
include <devkit_dims.scad>
include <board_dims.scad>
part = "assembly";

// ---- derived ----
side_x = 7;                                       // room for the lid pillars beside the board
cav_x0 = -side_x - slip;  cav_x1 = pcb_w + side_x + slip;
cav_y0 = -slip;           cav_y1 = pcb_h + slip;
board_z = 0;
tall = max([for (r = z_max_by_region) r[5]]);
lid_inner_z = pcb_t + max(tall, stack_h) + 1.5;
floor_z = -standoff - floor_t;
out_x0 = cav_x0 - wall; out_x1 = cav_x1 + wall; out_y0 = cav_y0 - wall; out_y1 = cav_y1 + wall;
groove_mouth = 10; groove_d = 3.2; groove_w = groove_mouth + 2 * groove_d;   // widest point (45° flanks)
housing_w = 14; housing_d = 4;
groove_xs = [out_x0 + 18, out_x1 - 18];
usb_y = u1_pin19[1] + usb_dy; usb_cz = pcb_t + usb_z;
sma_y = u1_pin1[1] + 4; sma_z = (floor_z + lid_inner_z) / 2;
pillar = 8;

module rbox(x0, y0, x1, y1, z0, z1, r = corner_r) {
  translate([0, 0, z0]) linear_extrude(z1 - z0) hull()
    for (x = [x0 + r, x1 - r], y = [y0 + r, y1 - r]) translate([x, y]) circle(r);
}
module dovetail_profile(mouth, depth, extra = 0) {  // trapezoid in XZ, mouth at z=0 widening to -depth
  polygon([[-mouth / 2 - extra, 0], [mouth / 2 + extra, 0], [mouth / 2 + depth + extra, -depth], [-mouth / 2 - depth - extra, -depth]]);
}
function conn_cut(c) = [c[1] - slip, c[2] + slip, pcb_t - 0.5, pcb_t + c[3] + slip];   // x0, x1, z0, z1

module base() {
  difference() {
    union() {
      rbox(out_x0, out_y0, out_x1, out_y1, floor_z, lid_inner_z);
      for (gx = groove_xs) translate([gx - housing_w / 2, out_y0, floor_z - housing_d]) cube([housing_w, out_y1 - out_y0, housing_d]);
      translate([(out_x0 + out_x1) / 2 - 7, out_y1 - 14, floor_z - housing_d]) cube([14, 10, housing_d]);
    }
    rbox(cav_x0, cav_y0, cav_x1, cav_y1, floor_z + floor_t, lid_inner_z + 1, 1);
    // connector cut-outs through the bottom wall
    for (c = conn) let(k = conn_cut(c)) translate([k[0], out_y0 - 1, k[2]]) cube([k[1] - k[0], wall + 2 + slip, k[3] - k[2]]);
    // USB through the left wall
    translate([out_x0 - 1, usb_y - usb_plug_w / 2, usb_cz - usb_plug_h / 2]) cube([wall + side_x + 2, usb_plug_w, usb_plug_h]);
    // SMA D-hole through the right wall
    translate([out_x1 - wall - 1, sma_y, sma_z]) rotate([0, 90, 0]) linear_extrude(wall + 2)
      intersection() { circle(d = 6.5); square([6.5, 5.8], center = true); }
    // DHT vents: bottom wall and right wall
    for (i = [0:5]) translate([dht[0] + 2 + i * 2.2, out_y0 - 1, floor_z + floor_t + 2]) cube([1.2, wall + 2, 12]);
    for (i = [0:5]) translate([out_x1 - wall - 1, dht[1] + 2 + i * 2.2, floor_z + floor_t + 2]) cube([wall + 2, 1.2, 12]);
    // dovetail grooves (open at the bottom end, closed 4 mm before the top)
    for (gx = groove_xs) translate([gx, out_y0 - 1, floor_z - housing_d]) rotate([-90, 0, 0]) mirror([0, 1, 0])
      linear_extrude(out_y1 - out_y0 - 4 + 1) dovetail_profile(groove_mouth, groove_d);
    // latch pocket
    translate([(out_x0 + out_x1) / 2 - 5, out_y1 - 12, floor_z - housing_d - 0.01]) cube([10, 3, 1.2]);
    // insert holes: PCB bosses and lid pillars
    for (h = holes) translate([h[0], h[1], -insert_h]) cylinder(d = insert_d, h = insert_h + 0.01);
    for (p = pillar_xy()) translate([p[0], p[1], lid_inner_z - insert_h]) cylinder(d = insert_d, h = insert_h + 0.01);
  }
  // PCB bosses
  for (h = holes) difference() {
    translate([h[0], h[1], floor_z + floor_t - 0.01]) cylinder(d = 7, h = standoff + 0.01);
    translate([h[0], h[1], -insert_h]) cylinder(d = insert_d, h = insert_h + 0.02);
  }
  // lid pillars, in the side_x strips
  for (p = pillar_xy()) difference() {
    translate([p[0] - pillar / 2, p[1] - pillar / 2, floor_z + floor_t - 0.01]) cube([pillar, pillar, lid_inner_z - floor_z - floor_t + 0.01]);
    translate([p[0], p[1], lid_inner_z - insert_h]) cylinder(d = insert_d, h = insert_h + 0.02);
  }
  // DHT baffle under the board (top and left edges of the keep-out)
  translate([dht[0] - 1.2, dht[1], floor_z + floor_t - 0.01]) cube([1.2, dht[3] - dht[1], standoff - 0.3]);
  translate([dht[0] - 1.2, dht[3], floor_z + floor_t - 0.01]) cube([dht[2] - dht[0] + 1.2, 1.2, standoff - 0.3]);
  echo("FIT", "cav_x0", cav_x0); echo("FIT", "cav_x1", cav_x1); echo("FIT", "cav_y0", cav_y0); echo("FIT", "cav_y1", cav_y1);
  echo("FIT", "board_z", board_z); echo("FIT", "lid_inner_z", lid_inner_z); echo("FIT", "groove_w", groove_w);
  echo("FIT", "usb_cut", [usb_y - usb_plug_w / 2, usb_y + usb_plug_w / 2, usb_cz - usb_plug_h / 2, usb_cz + usb_plug_h / 2]);
  for (c = conn) echo("FIT", str("conn_cut_", c[0]), conn_cut(c));
}
function pillar_xy() = [[cav_x0 + slip + side_x / 2, cav_y0 + 6], [cav_x1 - slip - side_x / 2, cav_y0 + 6],
                        [cav_x0 + slip + side_x / 2, cav_y1 - 6], [cav_x1 - slip - side_x / 2, cav_y1 - 6]];

module bracket() {
  plate_t = 3; tongue_t = 1.6;
  y0 = out_y0 + 5; y1 = out_y1 + 10;            // tab sticks 10 mm above the box
  rail_w = groove_w - 2 * dovetail;
  difference() {
    union() {
      rbox(out_x0, y0, out_x1, y1 - 10, 0, plate_t);
      for (gx = groove_xs) translate([gx, y0, plate_t]) rotate([-90, 0, 0]) mirror([0, 1, 0])
        linear_extrude(out_y1 - out_y0 - 4 - 5 - 1) mirror([0, 1, 0]) dovetail_profile(groove_mouth - 2 * dovetail, groove_d - 0.2);
    }
    // wall screw holes, countersunk
    for (y = [y0 + 15, y1 - 25]) translate([(out_x0 + out_x1) / 2, y, -0.01]) {
      cylinder(d = 4.5, h = plate_t + 1); translate([0, 0, plate_t - 2.2]) cylinder(d1 = 4.5, d2 = 9, h = 2.21);
    }
    // latch tongue: free on three sides
    lx = (out_x0 + out_x1) / 2;
    translate([lx - 6, out_y1 - 30, -0.01]) difference() { cube([12, 30 + 0.02, plate_t + 0.02]); translate([1, -1, 0]) cube([10, 31, plate_t + 1]); }
    translate([lx - 5, out_y1 - 29, -0.01]) cube([10, 30, plate_t - tongue_t + 0.01]);   // 1.4 mm air behind
  }
  lx = (out_x0 + out_x1) / 2;
  translate([lx - 5, out_y1 - 12, plate_t]) rotate([0, 90, 0]) linear_extrude(10) polygon([[0, 0], [-1.2, 1.5], [0, 3]]);  // ramp bump
  translate([lx - 5, out_y1 + 1, plate_t - tongue_t]) cube([10, 9, tongue_t]);       // release tab
  echo("FIT", "rail_w", rail_w);
}

if (part == "base") base();
else if (part == "bracket") bracket();
else if (part == "assembly") { color("gray") bracket(); color("white", 0.6) base(); }
```

  The rail must be the same dovetail profile as the groove, upside down on the plate's front
  face, and narrower by `dovetail` per side. Check this in the render in Step 6.

- [ ] **Step 5: Write `make_stl.sh`.**

```bash
#!/bin/sh
# Renders every printable part to stl/ and a PNG preview of each (plus the assembly) to renders/.
set -eu
cd "$(dirname "$0")"
OSCAD=${OSCAD:-/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD}
mkdir -p stl renders
PARTS=${*:-"bracket base lid btn_boot btn_en lightpipe fittest"}
for p in $PARTS; do
  "$OSCAD" -q -o "stl/$p.stl" -D "part=\"$p\"" enclosure.scad
  "$OSCAD" -q -o "renders/$p.png" --imgsize=1200,900 --viewall --autocenter --colorscheme=Tomorrow -D "part=\"$p\"" enclosure.scad
done
"$OSCAD" -q -o renders/assembly.png --imgsize=1600,1200 --viewall --autocenter --colorscheme=Tomorrow -D 'part="assembly"' enclosure.scad
ls -la stl renders
```

- [ ] **Step 6: Run the fit check and render.**
  - Run: `cd $REPO/hardware/enclosure && python3 check_fit.py && sh make_stl.sh bracket base; cd $REPO`
  - Expected: `FIT OK`. Then `stl/bracket.stl`, `stl/base.stl` and their PNGs, with no
    `WARNING: Object may not be a valid 2-manifold` lines.
  - Read `renders/base.png` and `renders/bracket.png`. The base should be an open box with three
    bottom slots, a left USB window, a right round hole and back grooves; the bracket should be a
    plate with two rails and a tongue.

- [ ] **Step 7: Commit.**

```bash
cd $REPO && git add hardware/enclosure && git commit -m "feat(enclosure): base and wall bracket with fit check and STL export"
```

---

### Task 7: Lid, buttons, light pipes, fit-test piece

**Files:**
- Modify: `hardware/enclosure/enclosure.scad`
- Modify: `hardware/enclosure/check_fit.py`

**Interfaces:**
- Consumes: Task 6's derived values (`pillar_xy()`, `lid_inner_z`, `out_*`).
- Produces: `lid`, `btn_boot`, `btn_en`, `lightpipe` and `fittest` parts, plus FIT echoes for
  `led_holes`, `btn_holes`, `lid_bottom_z`.

- [ ] **Step 1: Extend `check_fit.py` (failing).** Before the final `if fails:`, add:

```python
L = fit("lid")
check(L, "lid emitted no FIT values")
if L:
    for (name, x, y), (hx, hy) in zip(B["leds"], L["led_holes"]):
        check(abs(hx - x) <= 0.5 and abs(hy - y) <= 0.5, f"{name} LED hole at ({hx},{hy}), LED at ({x},{y})")
    for key, off, (hx, hy) in (("BOOT", D["btn_boot"], L["btn_holes"][0]), ("EN", D["btn_en"], L["btn_holes"][1])):
        bx, by = B["u1_pin19"][0] + off[0], B["u1_pin19"][1] + off[1]
        check(abs(hx - bx) <= 0.5 and abs(hy - by) <= 0.5, f"{key} button at ({hx},{hy}), switch at ({bx},{by})")
    check(abs(L["lid_bottom_z"] - f["lid_inner_z"]) < 1e-6, "lid does not sit on the base walls")
for prt in ("btn_boot", "btn_en", "lightpipe", "fittest"):
    with tempfile.TemporaryDirectory() as t:
        rc = subprocess.run([OSCAD, "-o", f"{t}/{prt}.stl", "-D", f'part="{prt}"', str(HERE / "enclosure.scad")],
                            capture_output=True).returncode
    check(rc == 0, f"{prt} failed to render")
```

  - Run: `python3 $REPO/hardware/enclosure/check_fit.py`
  - Expected: `FAIL: lid emitted no FIT values`.

- [ ] **Step 2: Add the lid and small parts to `enclosure.scad`.** Insert before the
  `if (part == ...)` chain:

```openscad
led_tube_od = 5.2; led_hole = 3.2;
btn_sleeve_od = 7; btn_d = 4.4; btn_rest_gap = 0.5;
function btn_xy(o) = [u1_pin19[0] + o[0], u1_pin19[1] + o[1]];

module label(t, x, y, s = 4) { translate([x, y, -0.01]) mirror([1, 0, 0]) linear_extrude(0.61) text(t, size = s, halign = "center", valign = "center", font = "Liberation Sans:style=Bold"); }

module lid() {   // modelled in place: bottom face at z = lid_inner_z. Printed upside down (outer face on the plate).
  translate([0, 0, lid_inner_z]) difference() {
    union() {
      rbox(out_x0, out_y0, out_x1, out_y1, 0, lid_t);
      // locating lip inside the base walls
      difference() { rbox(cav_x0 + slip, cav_y0 + slip, cav_x1 - slip, cav_y1 - slip, -2, 0, 1);
                     rbox(cav_x0 + slip + 1.6, cav_y0 + slip + 1.6, cav_x1 - slip - 1.6, cav_y1 - slip - 1.6, -2.1, 0.1, 1); }
      // LED tubes down to 1 mm above the LED dome (LED_D3.0mm dome ~5.3 mm above the board)
      for (l = leds) translate([l[1], l[2], -(lid_inner_z - pcb_t - 5.3 - 1)]) cylinder(d = led_tube_od, h = lid_inner_z - pcb_t - 5.3 - 1);
      // button sleeves
      for (o = [btn_boot, btn_en]) let(p = btn_xy(o)) translate([p[0], p[1], -6]) cylinder(d = btn_sleeve_od, h = 6);
      // DHT baffle from above (stops 1 mm over the parts)
      translate([dht[0] - 1.2, dht[1], -(lid_inner_z - pcb_t - 9)]) cube([1.2, dht[3] - dht[1], lid_inner_z - pcb_t - 9]);
    }
    for (l = leds) translate([l[1], l[2], -50]) cylinder(d = led_hole, h = 100);
    for (o = [btn_boot, btn_en]) let(p = btn_xy(o)) translate([p[0], p[1], -50]) cylinder(d = btn_d + 2 * slip, h = 100);
    let(p = btn_xy(btn_en)) translate([p[0], p[1], lid_t - 1.5]) cylinder(d = 9, h = 2);   // recess so EN can't be hit by accident
    for (p = pillar_xy()) translate([p[0], p[1], -1]) { cylinder(d = 3.4, h = lid_t + 2); translate([0, 0, lid_t - 1.7]) cylinder(d1 = 3.4, d2 = 6.6, h = 1.71); }
    for (i = [0:5]) translate([dht[0] + 2 + i * 2.2, dht[1] + 3, -1]) cube([1.2, dht[3] - dht[1] - 6, lid_t + 2]);  // DHT vents
    // debossed labels on the outer face (z = lid_t), 0.6 mm deep
    translate([0, 0, lid_t]) mirror([0, 0, 1]) {
      for (l = leds) label(l[0], l[1], l[2] - 5, 3.5);
      let(p = btn_xy(btn_boot)) label("BOOT", p[0] + 9, p[1], 3.5);
      let(p = btn_xy(btn_en)) label("RST", p[0] + 9, p[1], 3.5);
      label("GARAGE", (out_x0 + out_x1) / 2, out_y1 - 10, 7);
    }
  }
  echo("FIT", "lid_bottom_z", lid_inner_z);
  echo("FIT", "led_holes", [for (l = leds) [l[1], l[2]]]);
  echo("FIT", "btn_holes", [btn_xy(btn_boot), btn_xy(btn_en)]);
}

module button() {   // push-rod: cap above the lid, shaft through the sleeve, flange below keeps it captive
  shaft = lid_inner_z + lid_t - (pcb_t + btn_top_z) - btn_rest_gap;
  cylinder(d = btn_d, h = shaft);
  translate([0, 0, shaft]) cylinder(d = btn_d + 2.0, h = 1.2);     // cap (sits on the lid face)
  translate([0, 0, 6 + 0.2]) cylinder(d1 = btn_d, d2 = btn_d + 1.2, h = 0.8);   // snap flange: pushed through the sleeve once
}
module lightpipe() { cylinder(d = led_hole - 0.2, h = lid_t + 4); translate([0, 0, lid_t + 4]) cylinder(d = led_hole + 1.5, h = 0.8); }
module fittest() {   // a 40 mm slice of groove + rail + one connector cut-out
  intersection() { base(); translate([groove_xs[0] - 10, out_y0 - 1, floor_z - housing_d - 1]) cube([40, 30, 40]); }
  translate([0, -40, 0]) intersection() { bracket(); translate([groove_xs[0] - 10, out_y0, -1]) cube([40, 30, 20]); }
}
```

  Then extend the part chain:

```openscad
else if (part == "lid") lid();
else if (part == "btn_boot" || part == "btn_en") button();
else if (part == "lightpipe") lightpipe();
else if (part == "fittest") fittest();
```

  Replace the `assembly` line with:
  `else if (part == "assembly") { color("gray") bracket(); color("white", 0.6) base(); color("green") translate([0,0,0]) cube([pcb_w, pcb_h, pcb_t]); color("white", 0.4) translate([0,0,0.5]) lid(); }`

  The text labels are mirrored because the lid prints upside down. Check the label direction in
  `renders/lid.png` from above. If the labels read mirrored there, remove the `mirror([1,0,0])` in
  `label()`.

- [ ] **Step 3: Run the fit check and render everything.**
  - Run: `cd $REPO/hardware/enclosure && python3 check_fit.py && sh make_stl.sh; cd $REPO`
  - Expected: `FIT OK`, then 7 STLs and 8 PNGs.
  - Read `renders/lid.png` and `renders/assembly.png`. The lid should show 2 LED holes, 2 button
    holes (EN recessed), 4 countersunk screw holes, vents, and readable labels from outside.

- [ ] **Step 4: Commit.**

```bash
cd $REPO && git add hardware/enclosure && git commit -m "feat(enclosure): lid with LED tubes, captive buttons, light pipes and fit-test piece"
```

---

### Task 8: Documentation

**Files:**
- Create: `hardware/enclosure/README.md`
- Modify: `README.md` (repo root), `hardware/garage-opener-pcb/README.md`

- [ ] **Step 1: Write `hardware/enclosure/README.md`.** It covers:
  - the part list, with the STL names;
  - print settings: PLA, 0.2 mm, 4 walls, 20 % gyroid, no supports;
  - orientation:
    - base: open side up;
    - lid: outer face down;
    - bracket: flat, rails up;
    - buttons: cap down;
    - light pipes: flange down;
  - an optional filament change at the label layer (0.6 mm) for coloured text;
  - the hardware list (the same items as the BOM PDF "also needed");
  - assembly:
    1. Melt the inserts in with a soldering iron at 200 °C.
    2. Fit the pigtail bulkhead to the right wall, flat to the D.
    3. Screw the PCB onto the bosses.
    4. Clip the U.FL onto the devkit.
    5. Push the buttons through the lid sleeves from inside, until the flange clicks past.
    6. Screw the lid on.
    7. Screw the bracket to the wall with its tab up.
    8. Slide the box down until the latch clicks.
    9. Plug in the three cables from below.
  - removal: pull the plugs, push the tab towards the wall, and lift;
  - "Before printing the box":
    - measure the devkit and edit `devkit_dims.scad`;
    - print `fittest.stl` and adjust `params.scad`;
    - regenerate with `python3 export_board_dims.py && python3 check_fit.py && sh make_stl.sh`;
  - PLA warning: not in direct sun, not on the opener motor; PETG works with the same files;
  - an embedded `renders/assembly.png`.

- [ ] **Step 2: Link it from the root `README.md`.** Add to the hardware links line:
  `· [Enclosure (3D print)](hardware/enclosure/README.md)`. In
  `hardware/garage-opener-pcb/README.md`, add a line pointing to `../enclosure/`.

- [ ] **Step 3: Full verification.**

```bash
cd $REPO && python3 hardware/garage-opener-pcb/check_schematic.py && python3 hardware/garage-opener-pcb/check_pcb.py \
  && (cd hardware/enclosure && python3 test_board_dims.py && python3 check_fit.py) && git status --short
```

  Expected: `SCHEMATIC OK`, `PCB OK`, `BOARD DIMS OK`, `FIT OK`, and only the README changes left
  in the status.

- [ ] **Step 4: Commit.**

```bash
cd $REPO && git add README.md hardware && git commit -m "docs: enclosure print/assembly guide and hardware links"
```
