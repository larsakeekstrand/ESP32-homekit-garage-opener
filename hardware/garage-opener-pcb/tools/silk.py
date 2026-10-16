#!/usr/bin/env python3
"""Adds the board-level silkscreen labels and tidies two reference designators.
Run with KiCad's bundled Python; idempotent (removes earlier board-level F.Silkscreen texts first)."""
import sys
from pathlib import Path
import pcbnew as p
sys.path.insert(0, str(Path(__file__).resolve().parent))
from placement import ORIGIN, BOARD_W

PCB = Path(__file__).resolve().parent.parent / "garage-opener-pcb.kicad_pcb"
b = p.LoadBoard(str(PCB))
fp = {f.GetReference(): f for f in b.GetFootprints()}
old = [d for d in b.GetDrawings() if d.GetClass() == "PCB_TEXT" and d.GetLayer() == p.F_SilkS]
for d in old:
    b.Delete(d)
V = lambda x, y: p.VECTOR2I(p.FromMM(x), p.FromMM(y))

def pad(ref, n):
    v = fp[ref].FindPadByNumber(n).GetPosition(); return p.ToMM(v.x), p.ToMM(v.y)

def text(s, x, y, size=1.0):
    t = p.PCB_TEXT(b); t.SetText(s); t.SetLayer(p.F_SilkS); t.SetPosition(V(x, y))
    t.SetTextSize(V(size, size)); t.SetTextThickness(p.FromMM(0.15)); b.Add(t)

# Connectors: labels sit above the pin row (the connector body runs to the board edge below it).
x1, y1 = pad("J1", "1"); x2, _ = pad("J1", "2")
text("5V IN", (x1 + x2) / 2, y1 - 5.6); text("+", x1, y1 - 3.8); text("-", x2, y1 - 3.8)
for n, w in (("1", "COM"), ("2", "NO"), ("3", "NC")):
    x, y = pad("J2", n); text(w, x, y - 4.4)
x, y = pad("J3", "1"); text("REED", x + 2.54, y - 4.4)
for ref, s in (("D4", "RUN"), ("D5", "DOOR")):
    x, y = pad(ref, "1"); text(s, x + 1.27, y - 3.0)
text("garage-opener", ORIGIN[0] + 10, ORIGIN[1] + 57.0); text("rev 1", ORIGIN[0] + 10, ORIGIN[1] + 59.0)

# The labels above replace these references; U1's reference sat on top of K1's, so centre it in the devkit.
for ref in ("J1", "J2", "J3", "D4", "D5"):
    fp[ref].Reference().SetVisible(False)
fp["U1"].Reference().SetPosition(fp["U1"].GetCourtyard(p.F_CrtYd).BBox().Centre())
# R8..R11 sit 3-4 mm apart, rotated 90 deg: put each reference above its own part.
for ref in ("R8", "R9", "R10", "R11"):
    c = fp[ref].GetPosition(); fp[ref].Reference().SetPosition(p.VECTOR2I(c.x, c.y - p.FromMM(3.2)))
    fp[ref].Reference().SetTextAngleDegrees(0); fp[ref].Reference().SetTextSize(V(0.8, 0.8))

p.ZONE_FILLER(b).Fill(b.Zones())
b.Save(str(PCB)); print("silkscreen labels written")
