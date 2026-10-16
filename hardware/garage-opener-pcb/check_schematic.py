#!/usr/bin/env python3
"""Checks garage-opener-pcb.kicad_sch: nets, footprints, DNP straps, and the GPIO map in main/board.h.

Usage: python3 check_schematic.py [project-dir]   (default: this file's directory)
Requires kicad-cli on PATH. Exit 0 = OK.
"""
import re, subprocess, sys, tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

PROJ = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent).resolve()
REPO = PROJ.parent.parent
SCH = PROJ / "garage-opener-pcb.kicad_sch"
STOCK_FP = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints")

# Each set is one complete net: (reference, pin number).
EXPECTED = [
    {("J1","1"), ("F1","1")},                                         # raw 5 V in
    {("F1","2"), ("D1","2")},                                         # fused, to Schottky anode
    {("D1","1"), ("C1","1"), ("C2","1"), ("U1","19"), ("K1","5"), ("D2","1")},  # +5V (PWR_FLAGs are virtual, not in the netlist)
    {("U1","1"), ("R3","1"), ("R5","1"), ("U2","1"), ("C4","1"), ("R8","1"), ("R10","1")},     # +3V3
    {("J1","2"), ("C1","2"), ("C2","2"), ("U1","14"), ("U1","20"), ("U1","26"), ("Q1","2"),
     ("J3","2"), ("D3","2"), ("C3","2"), ("U2","4"), ("C4","2"), ("D4","1"), ("D5","1"),
     ("R2","2"), ("R9","2"), ("R11","2")},                                     # GND
    {("U1","29"), ("R1","1"), ("R2","1")},                            # GPIO5 relay drive
    {("R1","2"), ("Q1","1")},                                         # base
    {("Q1","3"), ("K1","2"), ("D2","2")},                             # coil low side
    {("K1","1"), ("J2","1")},                                         # COM
    {("K1","3"), ("J2","2")},                                         # NO
    {("K1","4"), ("J2","3")},                                         # NC
    {("J3","1"), ("D3","1"), ("R4","1")},                             # reed, cable side
    {("U1","32"), ("R4","2"), ("R3","2"), ("C3","1")},                # GPIO4 reed
    {("U1","22"), ("R5","2"), ("U2","2")},                            # GPIO22 DHT data
    {("U1","34"), ("R6","1")}, {("R6","2"), ("D4","2")},              # GPIO2 run LED
    {("U1","31"), ("R7","1")}, {("R7","2"), ("D5","2")},              # GPIO16 door LED
    {("U1","7"), ("R8","2"), ("R9","1")},                             # GPIO32 ID0
    {("U1","8"), ("R10","2"), ("R11","1")},                           # GPIO33 ID1
]
USED_U1 = {"1","7","8","14","19","20","22","26","29","31","32","34"}
# board.h define -> a node that must share the GPIO's net
BOARD_H = {"DOOR_RELAY_GPIO": ("R1","1"), "DOOR_SENSOR_GPIO": ("R4","2"),
           "DHT_SENSOR_GPIO": ("U2","2"), "RUNNING_LED_GPIO": ("R6","1"), "DOOR_LED_GPIO": ("R7","1")}
DNP = {"R8", "R10"}

fails = []
def check(cond, msg):
    if not cond: fails.append(msg)

with tempfile.TemporaryDirectory() as t:
    out = Path(t) / "net.xml"
    r = subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml",
                        "-o", str(out), str(SCH)], capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"netlist export failed:\n{r.stdout}{r.stderr}")
    root = ET.parse(out).getroot()

nets = [{(n.get("ref"), n.get("pin")) for n in net.iter("node")} for net in root.iter("net")]
net_of = {node: s for s in nets for node in s}
# KiCad suffixes pin names with "_<number>" when a symbol repeats a name (GND), e.g. "IO5_29".
func = {(n.get("ref"), n.get("pin")): re.sub(r"_" + n.get("pin") + "$", "", n.get("pinfunction") or "")
        for n in root.iter("node")}

for exp in EXPECTED:
    first = sorted(exp)[0]
    got = net_of.get(first, set())
    check(got == exp, f"net of {first}: expected {sorted(exp)}, got {sorted(got)}")

for p in map(str, range(1, 39)):
    if p not in USED_U1:
        check(net_of.get(("U1", p), {("U1", p)}) == {("U1", p)}, f"unused U1 pin {p} is connected")

defines = dict(re.findall(r"#define\s+(\w+_GPIO)\s+GPIO_NUM_(\d+)", (REPO / "main/board.h").read_text()))
for name, anchor in BOARD_H.items():
    gpio = defines.get(name)
    check(gpio is not None, f"{name} missing from main/board.h")
    pins = [k for k, f in func.items() if k[0] == "U1" and f == f"IO{gpio}"]
    check(len(pins) == 1 and anchor in net_of.get(pins[0], set()),
          f"{name}=GPIO{gpio}: U1 pin {pins} is not on the net of {anchor}")
reset = defines.get("RESET_GPIO")
check(reset == "0", f"RESET_GPIO is GPIO{reset}, schematic assumes the devkit Boot button on GPIO0")

for comp in root.iter("comp"):
    ref, fp = comp.get("ref"), (comp.findtext("footprint") or "")
    check(":" in fp, f"{ref} has no footprint")
    if ":" in fp:
        lib, name = fp.split(":", 1)
        base = PROJ / "lib/garage-opener.pretty" if lib == "garage-opener" else STOCK_FP / f"{lib}.pretty"
        check((base / f"{name}.kicad_mod").is_file(), f"{ref}: footprint {fp} not found")
    dnp = any(p.get("name") == "dnp" for p in comp.iter("property"))
    check(dnp == (ref in DNP), f"{ref}: dnp={dnp}, expected {ref in DNP}")

if fails:
    print("\n".join("FAIL: " + f for f in fails)); sys.exit(1)
print("SCHEMATIC OK")
