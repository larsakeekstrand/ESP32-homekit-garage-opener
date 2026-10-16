#!/usr/bin/env python3
"""Coordinate-flip and sanity tests for board_dims.scad. Run: python3 test_board_dims.py"""
import re, sys
from pathlib import Path
s = (Path(__file__).parent / "board_dims.scad").read_text()
def num(name): return float(re.search(rf"^{name}\s*=\s*([-\d.]+);", s, re.M).group(1))
def rows(name): return re.findall(r'\["(\w+)",([^\]]*)\]', re.search(rf"^{name}\s*=\s*\[(.*?)\];", s, re.M | re.S).group(1))
def vec(name): return [float(v) for v in re.search(rf"^{name}\s*=\s*\[([^\]]*)\]", s, re.M).group(1).split(",")]
fails = []
w, h = num("pcb_w"), num("pcb_h")
if (w, h) != (95.0, 70.0): fails.append(f"board {w}x{h}")
conn = {n: [float(v) for v in vals.split(",") if v.strip()] for n, vals in rows("conn")}
if not conn["J1"][0] < conn["J3"][0] < conn["J2"][0]: fails.append(f"connector order not J1<J3<J2 left→right: {conn}")
leds = {n: [float(v) for v in vals.split(",") if v.strip()] for n, vals in rows("leds")}
if not leds["RUN"][0] < leds["DOOR"][0]: fails.append("RUN must be left of DOOR")
if not all(10 < l[1] < 35 for l in leds.values()): fails.append(f"LEDs should be in the lower half (scad y up): {leds}")
if not vec("u1_pin1")[1] > 50: fails.append(f"U1 must be near the top (scad y up), got {vec('u1_pin1')}")
if not vec("u1_pin19")[0] < vec("u1_pin1")[0]: fails.append("U1 pin 19 (USB end) must be left of pin 1")
for n, (x0, x1, z) in conn.items():
    if not (5 < z < 20): fails.append(f"{n} height {z} outside 5..20 mm")
zall = [float(v.split(",")[-1]) for n, v in rows("z_max_by_region") if n == "all"]
if not zall or not (15 <= zall[0] <= 20): fails.append(f"tallest part height suspicious: {zall}")
if fails: print("\n".join("FAIL: " + f for f in fails)); sys.exit(1)
print("BOARD DIMS OK")
