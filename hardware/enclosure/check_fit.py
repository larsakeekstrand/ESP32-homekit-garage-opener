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
PLUG_MARGIN = 1.5   # plug body vs header body, each side
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
        # sized for the mating plug (wider/taller than the header), not just the header
        check(c and c[0] <= x0 - PLUG_MARGIN + 1e-6 and c[1] >= x1 + PLUG_MARGIN - 1e-6 and c[3] >= f["board_z"] + B["pcb_t"] + z + 3.0 - 1e-6,
              f"{name} cut-out {c} leaves no room for the plug around the header {x0}..{x1}, top z {z}")
    u = f.get("usb_cut")   # [y0, y1, z0, z1]
    uy = B["u1_pin19"][1] + D["usb_dy"]; uz = f["board_z"] + B["pcb_t"] + D["usb_z"]
    check(u and u[0] <= uy - D["usb_w"] / 2 and u[1] >= uy + D["usb_w"] / 2 and u[2] <= uz - D["usb_hgt"] / 2 and u[3] >= uz + D["usb_hgt"] / 2,
          f"USB cut-out {u} doesn't cover the devkit USB (y {uy}, z {uz})")
    tall = max(r[-1] for r in B["z_max_by_region"])
    # The devkit sits on its male header's plastic strip on top of the female socket.
    check(D.get("male_hdr_h", 0) >= 2.0, "devkit_dims.scad has no male_hdr_h: the devkit sits ~2.5 mm higher than the socket alone")
    seat = D["socket_h"] + D.get("male_hdr_h", 0) + D["devkit_t"]
    check(D["btn_top_z"] > seat and D["usb_z"] > seat and D["stack_h"] > seat, "devkit heights ignore the male header strip")
    check(f["lid_inner_z"] >= f["board_z"] + B["pcb_t"] + max(tall, D["stack_h"]) + 1.0,
          f"lid inner z {f['lid_inner_z']} < tallest part {max(tall, D['stack_h'])} + 1 mm")
    # 3: printed floor-down without supports: the back is one slab covering the whole footprint
    sl = f.get("back_slab")
    check(sl and abs(sl[0] - f["out"][0]) < 1e-6 and abs(sl[1] - f["out"][1]) < 1e-6 and abs(sl[2] - f["out"][2]) < 1e-6
          and abs(sl[3] - f["out"][3]) < 1e-6, f"back of the base is not one full slab ({sl} vs outline {f.get('out')}): needs supports")
    for i, (hx, hy) in enumerate(B["holes"]):
        check(any(abs(bx - hx) < 1e-6 and abs(by - hy) < 1e-6 for bx, by in f.get("bosses", [])), f"no PCB boss under H{i + 1}")

def downward_facets_above_bed(part, limit=-0.75, below=1e9):
    """Area (mm^2) of facets facing down steeper than ~49 deg that are not on the bed (and start less than `below`
    above it): what would need supports."""
    with tempfile.TemporaryDirectory() as t:
        subprocess.run([OSCAD, "-o", f"{t}/p.stl", "-D", f'part="{part}"', str(HERE / "enclosure.scad")], capture_output=True)
        tri = [tuple(map(float, m)) for m in re.findall(r"vertex\s+(\S+)\s+(\S+)\s+(\S+)", Path(f"{t}/p.stl").read_text())]
    bed = min(v[2] for v in tri); area = 0.0
    for i in range(0, len(tri), 3):
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri[i:i + 3]
        ux, uy, uz, vx, vy, vz = bx - ax, by - ay, bz - az, cx - ax, cy - ay, cz - az
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        n = (nx * nx + ny * ny + nz * nz) ** 0.5
        if n and nz / n < limit and bed + 0.05 < min(az, bz, cz) < bed + below: area += n / 2
    return area
# The base and the fittest print lying on their backs. Nothing in the back material (back slab + floor) may face
# down: that is where supports would end up inside slots you can't reach. (Bridges over the cut-outs in the side
# walls are higher up and print without supports.)
check(f and f.get("back"), "base reports no back thickness")
if f and f.get("back"):
    back_top = f["back"][1] - f["back"][0]
    for prt in ("base", "fittest"):
        ov = downward_facets_above_bed(prt, limit=-0.5, below=back_top)
        check(ov < 0.5, f"{prt} printed back-down has {ov:.0f} mm^2 of overhang in its back: needs supports")

# Keyholes: the box hangs on two pan-head wall screws. Each keyhole is a round hole the head passes through, a
# narrow slot above it towards the wall (the neck the head hooks behind) and a wider head channel behind the neck.
kh = f.get("keyholes") if f else None
check(kh and len(kh) == 2, f"expected two keyholes, got {kh}")
if kh:
    hole_d, slot_w, neck_t, chan_d = f["kh_dims"]
    sd, hd, hh = PR["wall_screw_d"], PR["wall_screw_head_d"], PR["wall_screw_head_h"]
    check(slot_w >= sd + 0.3, f"keyhole slot {slot_w} too narrow for the {sd} mm screw")
    check((hd - slot_w) / 2 >= 1.5, f"keyhole slot {slot_w}: the {hd} mm head hooks only {(hd - slot_w) / 2:.2f} mm each side")
    check(hole_d >= hd + 0.6, f"keyhole hole {hole_d} too small for the {hd} mm head")
    check(neck_t >= 2.0, f"keyhole neck {neck_t} mm is too thin to hang the box on")
    check(chan_d >= hh + 0.5, f"keyhole head channel {chan_d} mm deep, the head is {hh} mm tall")
    (x0, yh0, yr0), (x1, yh1, yr1) = kh
    check(abs(yh0 - yh1) < 1e-6 and abs(yr0 - yr1) < 1e-6 and abs(x1 - x0) >= 40, f"keyholes {kh} must be level and >= 40 mm apart")
    for x, yh, yr in kh:
        check(yr - yh >= (hole_d + hd) / 2, f"keyhole at x={x}: the resting head overlaps the hole (travel {yr - yh})")
        for bx, by in B["holes"]:   # the head channel opens into the cavity: keep it clear of the 7 mm PCB bosses
            dx = max(0, abs(bx - x) - hole_d / 2); dy = max(0, yh - hole_d / 2 - by, by - yr - hole_d / 2)
            check((dx * dx + dy * dy) ** 0.5 >= 3.5 + 0.5, f"keyhole at x={x} runs into the PCB boss at ({bx},{by})")
        check(not (B["dht"][0] - 1.2 - hole_d / 2 < x < B["dht"][2] + hole_d / 2 and yh - hole_d / 2 < B["dht"][3] + 1.2),
              f"keyhole at x={x} runs into the DHT baffle")
    T = fit("template")
    check(T and sorted(map(tuple, T.get("holes", []))) == sorted((x, yr) for x, _, yr in kh),
          f"drilling template holes {T.get('holes')} don't match where the screws rest {[(x, yr) for x, _, yr in kh]}")

L = fit("lid")
check(L, "lid emitted no FIT values")
if L and f:
    for (name, x, y), (hx, hy) in zip(B["leds"], L["led_holes"]):
        check(abs(hx - x) <= 0.5 and abs(hy - y) <= 0.5, f"{name} LED hole at ({hx},{hy}), LED at ({x},{y})")
    for key, off, (hx, hy) in (("BOOT", D["btn_boot"], L["btn_holes"][0]), ("EN", D["btn_en"], L["btn_holes"][1])):
        bx, by = B["u1_pin19"][0] + off[0], B["u1_pin19"][1] + off[1]
        check(abs(hx - bx) <= 0.5 and abs(hy - by) <= 0.5, f"{key} button at ({hx},{hy}), switch at ({bx},{by})")
    check(abs(L["lid_bottom_z"] - f["lid_inner_z"]) < 1e-6, "lid does not sit on the base walls")
    check(L.get("lip_bottom_z", -99) >= B["pcb_t"] + max(tall, D["stack_h"]) + 0.5 - 1e-6, f"lid lip reaches down to {L.get('lip_bottom_z')}, tallest part is at {B['pcb_t'] + max(tall, D['stack_h'])}")
    # The lid screws clamp the PCB: one tube per hole, ending exactly on the board's top face.
    check(sorted(map(tuple, L["tubes"])) == sorted(map(tuple, B["holes"])), f"lid tubes {L['tubes']} not over the holes {B['holes']}")
    check(abs(L["tube_bottom_z"] - B["pcb_t"]) < 1e-6, f"lid tubes end at z={L['tube_bottom_z']}, board top is {B['pcb_t']}")
    tip = L.get("screw_tip_z", 99)   # inserts sit in the bosses from z = -5 (bottom of a 5 mm insert) to z = 0
    check(-5.0 + 0.5 <= tip <= -3.0, f"M3 x {PR.get('lid_screw_len')} lid screw tip at z={tip}: needs 3..4.5 mm of thread in the 5 mm insert")
    # Button sleeves must stay above the devkit's tallest part; rods must reach the switches.
    check(L["sleeve_bottom_z"] >= B["pcb_t"] + D["stack_h"] + 0.3, f"button sleeves reach down to {L['sleeve_bottom_z']}, devkit top is {B['pcb_t'] + D['stack_h']}")
    for key in ("btn_boot", "btn_en"):
        bf = fit(key)
        check(abs(bf["rod_bottom_z"] - (B["pcb_t"] + D["btn_top_z"] + 0.5)) < 1e-6, f"{key} rod bottom {bf.get('rod_bottom_z')} is not 0.5 mm above the switch")
        # 1: pressing the cap must be able to move the rod past the 0.5 mm rest gap and through the switch travel
        check(bf.get("cap_bottom_z", 0) - bf.get("face_z", 0) >= 0.5 + 1.0, f"{key}: cap rests {bf.get('cap_bottom_z', 0) - bf.get('face_z', 0):.2f} mm above the lid, the rod can't reach the switch")
for prt in ("btn_boot", "btn_en", "lightpipe", "fittest", "template"):
    with tempfile.TemporaryDirectory() as t:
        rc = subprocess.run([OSCAD, "-o", f"{t}/{prt}.stl", "-D", f'part="{prt}"', str(HERE / "enclosure.scad")],
                            capture_output=True).returncode
    check(rc == 0, f"{prt} failed to render")

if fails:
    print("\n".join("FAIL: " + x for x in fails)); sys.exit(1)
print("FIT OK")
