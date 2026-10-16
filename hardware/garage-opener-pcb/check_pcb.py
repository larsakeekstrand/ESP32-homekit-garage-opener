#!/usr/bin/env python3
"""Checks garage-opener-pcb.kicad_pcb: locked placement, connector/USB orientation, DRC and parity.

Usage: python3 check_pcb.py [--placement-only]     Requires KiCad 10 (bundled Python, kicad-cli).
"""
import json, subprocess, sys, tempfile
from pathlib import Path

PROJ = Path(__file__).resolve().parent
PCB = PROJ / "garage-opener-pcb.kicad_pcb"
PYK = "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3"
sys.path.insert(0, str(PROJ / "tools"))
from placement import BOARD_W, BOARD_H, ORIGIN, PLACE, DHT_KEEPOUT  # noqa: E402

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
texts = [d.GetText() for d in b.GetDrawings() if isinstance(d, p.PCB_TEXT)]
out["texts"] = texts
print(json.dumps(out))
'''

fails = []
def check(cond, msg):
    if not cond: fails.append(msg)

if not PCB.is_file():
    print(f"FAIL: {PCB.name} does not exist"); sys.exit(1)
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
        check(f["crt"][1] < f["pads"]["1"][1] - 0.1 and f["crt"][3] - f["pads"]["1"][1] > 5,
              f"{ref} body does not extend downwards from its pins: opening faces inwards")

# Devkit: J2 = pins 1..19 (3V3 at the module end .. 5V at the USB end), J3 = pins 20..38 (GND at the module end
# .. CLK at the USB end). The USB end (pins 19/38) must be on the left; non-mirrored, J2 is then the top row.
u = fp.get("U1")
if u:
    p1, p19, p20, p38 = (rel(*u["pads"][n][:2]) for n in ("1", "19", "20", "38"))
    check(p19[0] < p1[0] and p38[0] < p20[0], "U1 USB end (pins 19/38) is not on the left")
    check(abs(p19[1] - p1[1]) < 0.01 and abs(p20[1] - p38[1]) < 0.01, "U1 rows are not horizontal")
    check(p1[1] < p20[1], "U1 is mirrored: the pin 1..19 row must be the top row")
    check(u["crt"][0] - ox >= -0.06, "U1 USB end is outside the board")

# DHT keep-out exists where placement.py says.
check(any(all(abs(a - b) < 0.2 for a, b in zip(rel(k[0], k[1]) + rel(k[2], k[3]), DHT_KEEPOUT)) for k in g["keepouts"]),
      "DHT22 keep-out rule area missing or misplaced")

def drc_rules_load():
    """KiCad silently ignores the whole .kicad_dru when any rule fails to parse. Append a rule that must fire
    (every track narrower than 50 mm) in a scratch copy of the project and confirm DRC reports it."""
    import shutil
    with tempfile.TemporaryDirectory() as tmp:
        for ext in ("kicad_pcb", "kicad_pro", "kicad_dru"):
            shutil.copy(PROJ / f"garage-opener-pcb.{ext}", Path(tmp) / f"garage-opener-pcb.{ext}")
        dru = Path(tmp) / "garage-opener-pcb.kicad_dru"
        dru.write_text(dru.read_text() + "(rule \"probe\" (condition \"A.Type == 'Track'\") (constraint track_width (min 50mm)))\n")
        rpt = Path(tmp) / "probe.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--severity-error", "--format", "json", "-o", str(rpt),
                        str(Path(tmp) / "garage-opener-pcb.kicad_pcb")], capture_output=True)
        return any(v["type"] == "track_width" for v in json.loads(rpt.read_text())["violations"])

if "--placement-only" not in sys.argv:
    check(drc_rules_load(), "garage-opener-pcb.kicad_dru does not parse: KiCad ignores all custom rules")
    with tempfile.TemporaryDirectory() as tmp:
        rpt = Path(tmp) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--schematic-parity", "--severity-error", "--format", "json",
                        "-o", str(rpt), str(PCB)], capture_output=True, text=True)
        d = json.loads(rpt.read_text())
    errs = [v for v in d.get("violations", []) if v.get("severity") == "error"]
    unc = d.get("unconnected_items", [])
    par = d.get("schematic_parity", [])
    for v in errs[:15]:
        fails.append(f"DRC {v['type']}: {v['description']} — " + "; ".join(i['description'] for i in v.get('items', [])))
    check(not errs, f"{len(errs)} DRC errors")
    check(not unc, f"{len(unc)} unconnected items")
    check(not par, f"{len(par)} schematic parity issues: " + "; ".join(x.get('description', '') for x in par[:5]))

if fails:
    print("\n".join("FAIL: " + f for f in fails)); sys.exit(1)
print("PCB OK" + (" (placement only)" if "--placement-only" in sys.argv else ""))
