#!/usr/bin/env python3
"""Routes garage-opener-pcb.kicad_pcb: exports Specctra DSN, runs Freerouting headless, imports SES, refills zones.
Run with KiCad's bundled Python. Existing locked tracks are kept (exported as protected wires).
FR_JAR overrides the Freerouting jar (it must run on the installed Java)."""
import os, subprocess, sys
from pathlib import Path
import pcbnew as p

PROJ = Path(__file__).resolve().parent.parent
PCB = PROJ / "garage-opener-pcb.kicad_pcb"
JAR = os.environ.get("FR_JAR", str(Path.home() / ".local/share/freerouting/freerouting.jar"))
PASSES = os.environ.get("FR_PASSES", "30")
WORK = PROJ / "tools/.route"; WORK.mkdir(exist_ok=True)
dsn, ses = WORK / "board.dsn", WORK / "board.ses"
ses.unlink(missing_ok=True)

b = p.LoadBoard(str(PCB))
for z in b.Zones():                 # drop unconnected copper fragments of the GND pours
    if not z.GetIsRuleArea(): z.SetIslandRemovalMode(p.ISLAND_REMOVAL_MODE_ALWAYS)


def stitch(board):
    """Adds a GND via inside every F.Cu GND island that holds no through-hole GND pad (it would otherwise
    be isolated): the via lands where the island and the B.Cu pour both have room for it."""
    gnd = board.FindNet("GND"); added = 0
    pours = {z.GetFirstLayer(): z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == "GND"}
    top, bot = (pours[l].GetFilledPolysList(l) for l in (p.F_Cu, p.B_Cu))
    room = p.FromMM(0.3 + 0.35)                     # via radius + clearance margin
    tht = [pd.GetPosition() for f in board.GetFootprints() for pd in f.Pads()
           if pd.GetNetname() == "GND" and pd.GetAttribute() == p.PAD_ATTRIB_PTH]
    for i in range(top.OutlineCount()):
        isl = p.SHAPE_POLY_SET(); isl.AddOutline(top.Outline(i))
        if any(isl.Contains(pt) for pt in tht): continue
        a = isl.CloneDropTriangulation(); a.Inflate(-room, p.CORNER_STRATEGY_ROUND_ALL_CORNERS, p.FromMM(0.01))
        bb = bot.CloneDropTriangulation(); bb.Inflate(-room, p.CORNER_STRATEGY_ROUND_ALL_CORNERS, p.FromMM(0.01))
        a.BooleanIntersection(bb)
        if a.OutlineCount() == 0:
            print("WARNING: no room for a stitching via in island", i); continue
        v = p.PCB_VIA(board); v.SetPosition(a.Outline(0).Centre() if a.Contains(a.Outline(0).Centre()) else a.Outline(0).CPoint(0))
        v.SetWidth(p.FromMM(0.6)); v.SetDrill(p.FromMM(0.3)); v.SetNet(gnd); board.Add(v); added += 1
    return added

if "--refill-only" in sys.argv:
    p.ZONE_FILLER(b).Fill(b.Zones())
    n = stitch(b); p.ZONE_FILLER(b).Fill(b.Zones())
    b.Save(str(PCB)); sys.exit(print("zones refilled,", n, "stitching vias added"))
for z in b.Zones():                 # copper pours confuse the autorouter; unfill before export
    if not z.GetIsRuleArea(): z.UnFill()
assert p.ExportSpecctraDSN(b, str(dsn)), "DSN export failed"
r = subprocess.run(["java", "-Djava.awt.headless=true", "-jar", JAR, "-de", str(dsn), "-do", str(ses), "-mp", PASSES],
                   capture_output=True, text=True, timeout=1200)
print(r.stdout[-1500:], r.stderr[-1500:])
assert ses.is_file(), "Freerouting produced no SES"
assert p.ImportSpecctraSES(b, str(ses)), "SES import failed"
p.ZONE_FILLER(b).Fill(b.Zones())
print(stitch(b), "stitching vias added"); p.ZONE_FILLER(b).Fill(b.Zones())
b.Save(str(PCB))
print("routed", sum(1 for _ in b.GetTracks()), "track segments/vias")
