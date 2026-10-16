#!/usr/bin/env python3
"""Numeric fit check: runs OpenSCAD on enclosure.scad, collects echo("FIT", ...) values, and compares them with
board_dims.scad, devkit_dims.scad and params.scad. Run: python3 check_fit.py   Exit 0 = OK."""
import ast, re, shutil, subprocess, sys, tempfile
from pathlib import Path
HERE = Path(__file__).resolve().parent
OSCAD = shutil.which("openscad") or "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"

def lit(s):
    return ast.literal_eval(s.replace("true", "True").replace("false", "False"))

def fit(part):
    """Echoed FIT values of one part (rendering only the echo output, not the geometry)."""
    with tempfile.TemporaryDirectory() as t:
        out = Path(t) / "x.echo"
        r = subprocess.run([OSCAD, "-o", str(out), "-D", f'part="{part}"', str(HERE / "enclosure.scad")],
                           capture_output=True, text=True)
        txt = out.read_text() if out.exists() else ""
    if r.returncode != 0:
        sys.exit(f"openscad failed for {part}:\n{r.stderr[-1500:]}")
    return {m.group(1): lit(m.group(2)) for m in re.finditer(r'ECHO: "FIT", "(\w+)", (.+)', txt)}

# eval() runs only on this repo's own .scad files, with builtins removed; the values are plain arithmetic.
def scad_vals(name):
    """Top-level `name = expr;` assignments of a .scad file, evaluated in order (plain arithmetic only)."""
    env = {}
    for m in re.finditer(r"^(\w+)\s*=\s*(.+?);", (HERE / name).read_text(), re.M):
        try:
            env[m.group(1)] = eval(m.group(2).replace("true", "True").replace("false", "False"), {"__builtins__": {}}, dict(env))
        except Exception:
            pass
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
    for i, (hx, hy) in enumerate(B["holes"]):
        check(any(abs(bx - hx) < 1e-6 and abs(by - hy) < 1e-6 for bx, by in f.get("bosses", [])), f"no PCB boss under H{i + 1}")

g = fit("bracket")
check(g.get("rail_w") and f and abs(g["rail_w"] - (f["groove_w"] - 2 * PR["dovetail"])) < 1e-6,
      f"bracket rail {g.get('rail_w')} vs base groove {f.get('groove_w') if f else None}: clearance must be {PR['dovetail']} per side")

if fails:
    print("\n".join("FAIL: " + x for x in fails)); sys.exit(1)
print("FIT OK")
