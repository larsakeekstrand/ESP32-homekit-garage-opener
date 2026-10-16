#!/usr/bin/env python3
"""One-shot: builds garage-opener-pcb.kicad_pcb from the schematic netlist + tools/placement.py.
Run with KiCad's bundled Python. Refuses to overwrite an existing board unless --force (the board is the
source of truth once routed)."""
import json, subprocess, sys, tempfile
import xml.etree.ElementTree as ET  # input is the netlist kicad-cli just wrote from our own schematic
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
def nc(name, w, clr):
    return {"name": name, "track_width": w, "clearance": clr, "via_diameter": 0.6, "via_drill": 0.3,
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
comps = {c.get("ref"): c for c in root.iter("comp")}
pin_net = {(n.get("ref"), n.get("pin")): net.get("name") for net in root.iter("net") for n in net.iter("node")}
assert set(comps) == set(PLACE), f"placement/netlist mismatch: {set(comps) ^ set(PLACE)}"

board = p.NewBoard(str(PCB))
nets = {}
for name in sorted(set(pin_net.values())):
    ni = p.NETINFO_ITEM(board, name); board.Add(ni); nets[name] = ni

# 3. Footprints
for ref, comp in sorted(comps.items()):
    lib, name = comp.findtext("footprint").split(":", 1)
    libdir = PROJ / "lib/garage-opener.pretty" if lib == "garage-opener" else STOCK / f"{lib}.pretty"
    f = p.FootprintLoad(str(libdir), name)
    assert f, f"{lib}:{name}"
    f.SetFPID(p.LIB_ID(lib, name)); f.SetReference(ref); f.SetValue(comp.findtext("value"))
    x, y, rot, locked = PLACE[ref]
    f.SetPosition(P(x, y)); f.SetOrientationDegrees(rot)
    for pad in f.Pads():
        net = pin_net.get((ref, pad.GetNumber()))
        if net: pad.SetNet(nets[net])
    # Parity: link to the schematic symbol by its sheet path + symbol UUID
    f.SetPath(p.KIID_PATH(comp.find("sheetpath").get("tstamps") + comp.findtext("tstamps")))
    if any(pr.get("name") == "dnp" for pr in comp.iter("property")):
        f.SetDNP(True)
    board.Add(f); f.SetLocked(locked)

# 4. Board edge (Edge.Cuts rectangle), DHT keep-out, GND pours on both layers
def seg(a, b, layer):
    s = p.PCB_SHAPE(board); s.SetShape(p.SHAPE_T_SEGMENT); s.SetStart(a); s.SetEnd(b); s.SetLayer(layer)
    s.SetWidth(p.FromMM(0.1)); board.Add(s)
c = [P(0, 0), P(BOARD_W, 0), P(BOARD_W, BOARD_H), P(0, BOARD_H)]
for i in range(4): seg(c[i], c[(i + 1) % 4], p.Edge_Cuts)

def zone(rect, layerset, rule_area, net=None):
    z = p.ZONE(board); z.SetLayerSet(layerset)
    x0, y0, x1, y1 = rect
    ol = z.Outline(); ol.NewOutline()
    for (x, y) in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)): ol.Append(P(x, y))
    if rule_area:
        z.SetIsRuleArea(True); z.SetDoNotAllowFootprints(True); z.SetDoNotAllowTracks(False)
        z.SetDoNotAllowVias(False); z.SetDoNotAllowPads(False); z.SetDoNotAllowZoneFills(False); z.SetZoneName("DHT22 body")
    else:
        z.SetNet(net); z.SetIslandRemovalMode(p.ISLAND_REMOVAL_MODE_ALWAYS); z.SetLocalClearance(p.FromMM(0.3)); z.SetMinThickness(p.FromMM(0.25))
        z.SetPadConnection(p.ZONE_CONNECTION_THERMAL); z.SetThermalReliefGap(p.FromMM(0.4))
        z.SetThermalReliefSpokeWidth(p.FromMM(0.6))
    board.Add(z); return z
ls = p.LSET(); ls.AddLayer(p.F_Cu)
zone(DHT_KEEPOUT, ls, True)
for layer in (p.F_Cu, p.B_Cu):
    l1 = p.LSET(); l1.AddLayer(layer)
    zone((0.5, 0.5, BOARD_W - 0.5, BOARD_H - 0.5), l1, False, nets["GND"])

board.Save(str(PCB))
print("wrote", PCB.name, len(comps), "footprints")
