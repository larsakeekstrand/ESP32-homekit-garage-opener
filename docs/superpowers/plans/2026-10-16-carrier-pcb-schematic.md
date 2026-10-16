# Carrier-PCB Schematic (rev 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce an ERC-clean KiCad 10 schematic (with footprints, BOM and PDF) for the ESP32 DevKitC V4 carrier PCB, matching the firmware's GPIO map exactly.

**Architecture:** A project-local KiCad library holds the DevKitC V4 symbol and its two-row socket footprint. A throwaway Python generator (kept in the session scratchpad, not the repo) writes the `.kicad_sch` once from stock KiCad symbols, connecting pins with short wire stubs + net labels / power symbols. A committed checker script exports the netlist with `kicad-cli` and asserts every net, every footprint, and the GPIO map from `main/board.h`. After generation, the `.kicad_sch` is the source of truth and is edited in KiCad.

**Tech Stack:** KiCad 10.0.6 (`kicad-cli` at `/opt/homebrew/bin/kicad-cli`, stock libs under `/Applications/KiCad/KiCad.app/Contents/SharedSupport/`), Python 3 (stdlib only).

**Spec:** `docs/superpowers/specs/2026-10-16-carrier-pcb-schematic-design.md`

## Global Constraints

- Firmware unchanged: GPIO5 relay, GPIO4 reed, GPIO22 DHT, GPIO2 run LED, GPIO16 door LED, GPIO0 boot button (devkit), GPIO32/33 board ID (unused by firmware).
- Devkit: ESP32 DevKitC V4 (WROOM-32U), 38 pins, rows **25.4 mm** apart; left row U1 pins 1–19 = 3V3, EN, VP, VN, IO34, IO35, IO32, IO33, IO25, IO26, IO27, IO14, IO12, GND, IO13, SD2, SD3, CMD, 5V; right row U1 pins 20–38 = GND, IO23, IO22, TX, RX, IO21, GND, IO19, IO18, IO5, IO17, IO16, IO4, IO0, IO2, IO15, SD1, SD0, CLK.
- Passives 0805; screw terminals Phoenix MKDS-1,5 5.0 mm; LEDs 3 mm THT.
- Board ID rev 1 = `00`: R9, R11 (to GND) fitted; R8, R10 (to +3V3) DNP.
- ERC: zero errors. Every symbol has a footprint that resolves.
- Commits on local `main`, no push. Commit messages: no `Co-Authored-By` trailer (user's global CLAUDE.md).
- The generator script lives in the session scratchpad, never in the repo.

## Review Focus

1. **Relay contact mapping (NO vs NC).** KiCad's `SANYOU_SRD_Form_C` symbol is read here as 1 = COM, 3 = NO, 4 = NC, 2/5 = coil. If that is wrong, the door would be "held pressed" at rest. Task 2 Step 1 checks the symbol graphic and the footprint against the Sanyou/Songle SRD datasheet before the checker's expectations are trusted.
2. **Devkit pin numbering vs. physical rows.** A mirrored row puts 5 V on a GPIO. Task 1's test asserts names per pin number and pad positions (left row x = 0, right row x = 25.4 mm, pin 1 and pin 20 at the top).
3. **Unused devkit pins accidentally joined.** Labels with the same name merge nets silently. The checker asserts every unused U1 pin is alone in its net.
4. **DNP parts leaking into the BOM / ID straps both fitted.** The checker asserts R8/R10 are DNP and R9/R11 are not; the BOM test asserts R8/R10 are absent.
5. **Firmware drift.** If `main/board.h` changes later, the checker reads it at run time and fails when a GPIO no longer lands on the expected net.

---

### Task 1: Project skeleton + DevKitC V4 symbol and footprint library

**Files:**
- Create: `hardware/garage-opener-pcb/garage-opener-pcb.kicad_pro`
- Create: `hardware/garage-opener-pcb/sym-lib-table`
- Create: `hardware/garage-opener-pcb/fp-lib-table`
- Create: `hardware/garage-opener-pcb/lib/garage-opener.kicad_sym`
- Create: `hardware/garage-opener-pcb/lib/garage-opener.pretty/ESP32-DevKitC-V4_Socket.kicad_mod`
- Create (scratchpad, not committed): `$SCRATCH/gen_lib.py`, `$SCRATCH/test_lib.py`

where `SCRATCH=/private/tmp/claude-501/-Users-laek-source-ESP32-homekit-garage-opener/e6671cc7-40b5-4457-941a-8bc30d4d3b64/scratchpad`.

**Interfaces:**
- Produces: library nickname `garage-opener`; symbol `garage-opener:ESP32-DevKitC-V4` (38 pins, numbering per Global Constraints; 3V3 = `power_out`, 5V and GND = `power_in`, others `bidirectional`); footprint `garage-opener:ESP32-DevKitC-V4_Socket` (pads "1"–"38").

- [ ] **Step 1: Write the failing library test**

`$SCRATCH/test_lib.py`:

```python
#!/usr/bin/env python3
"""Asserts the project-local DevKitC V4 symbol + footprint are correct and load in KiCad."""
import re, subprocess, sys, tempfile
from pathlib import Path

P = Path(sys.argv[1])  # hardware/garage-opener-pcb
LEFT = ["3V3","EN","VP","VN","IO34","IO35","IO32","IO33","IO25","IO26","IO27","IO14","IO12",
        "GND","IO13","SD2","SD3","CMD","5V"]
RIGHT = ["GND","IO23","IO22","TX","RX","IO21","GND","IO19","IO18","IO5","IO17","IO16","IO4",
         "IO0","IO2","IO15","SD1","SD0","CLK"]

sym = (P / "lib/garage-opener.kicad_sym").read_text()
pins = re.findall(r'\(pin (\w+) line\s*\(at ([-\d.]+) ([-\d.]+) (\d+)\).*?\(name "([^"]*)".*?\(number "(\d+)"', sym, re.S)
got = {int(n): (name, typ, float(x)) for typ, x, y, a, name, n in pins}
assert len(got) == 38, f"expected 38 pins, got {len(got)}"
for i, name in enumerate(LEFT + RIGHT, start=1):
    assert got[i][0] == name, f"pin {i}: {got[i][0]} != {name}"
    assert (got[i][2] < 0) == (i <= 19), f"pin {i} on wrong side of symbol"
types = {got[i][0]: got[i][1] for i in got}
assert types["3V3"] == "power_out" and types["5V"] == "power_in" and types["GND"] == "power_in"

fp = (P / "lib/garage-opener.pretty/ESP32-DevKitC-V4_Socket.kicad_mod").read_text()
pads = {int(n): (float(x), float(y)) for n, x, y in
        re.findall(r'\(pad "(\d+)" thru_hole \w+\s*\(at ([-\d.]+) ([-\d.]+)', fp)}
assert sorted(pads) == list(range(1, 39)), "pads must be 1..38"
for i in range(1, 20):
    assert pads[i] == (0.0, round((i - 1) * 2.54, 2)), f"pad {i} at {pads[i]}"
    assert pads[i + 19] == (25.4, round((i - 1) * 2.54, 2)), f"pad {i+19} at {pads[i+19]}"

with tempfile.TemporaryDirectory() as t:
    for cmd in (["kicad-cli", "sym", "export", "svg", "-o", t, str(P / "lib/garage-opener.kicad_sym")],
                ["kicad-cli", "fp", "export", "svg", "-o", t, str(P / "lib/garage-opener.pretty")]):
        r = subprocess.run(cmd, capture_output=True, text=True)
        assert r.returncode == 0, f"{' '.join(cmd)} failed:\n{r.stdout}{r.stderr}"
    assert list(Path(t).glob("*.svg")), "no SVG exported"
print("LIB OK")
```

- [ ] **Step 2: Run it to verify it fails**

Run: `python3 $SCRATCH/test_lib.py hardware/garage-opener-pcb`
Expected: FAIL with `FileNotFoundError` for `lib/garage-opener.kicad_sym`.

- [ ] **Step 3: Write the library generator**

`$SCRATCH/gen_lib.py`:

```python
#!/usr/bin/env python3
"""Throwaway: writes the project-local DevKitC V4 symbol + footprint and the project skeleton."""
import json, sys, uuid
from pathlib import Path

P = Path(sys.argv[1]); (P / "lib/garage-opener.pretty").mkdir(parents=True, exist_ok=True)
NS = uuid.UUID("6f1c3c1e-6a43-4c1f-9a5e-3b0b8f0c2a11")
uid = lambda *k: str(uuid.uuid5(NS, "lib/" + "/".join(map(str, k))))
LEFT = ["3V3","EN","VP","VN","IO34","IO35","IO32","IO33","IO25","IO26","IO27","IO14","IO12",
        "GND","IO13","SD2","SD3","CMD","5V"]
RIGHT = ["GND","IO23","IO22","TX","RX","IO21","GND","IO19","IO18","IO5","IO17","IO16","IO4",
         "IO0","IO2","IO15","SD1","SD0","CLK"]
FONT = '(effects (font (size 1.27 1.27)))'
HIDDEN = '(hide yes) (effects (font (size 1.27 1.27)))'

def ptype(n):
    return {"3V3": "power_out", "5V": "power_in", "GND": "power_in"}.get(n, "bidirectional")

pins = []
for i, n in enumerate(LEFT):
    pins.append(f'(pin {ptype(n)} line (at -15.24 {22.86 - i*2.54:.2f} 0) (length 2.54) '
                f'(name "{n}" {FONT}) (number "{i+1}" {FONT}))')
for i, n in enumerate(RIGHT):
    pins.append(f'(pin {ptype(n)} line (at 15.24 {22.86 - i*2.54:.2f} 180) (length 2.54) '
                f'(name "{n}" {FONT}) (number "{i+20}" {FONT}))')

sym = f'''(kicad_symbol_lib (version 20241209) (generator "kicad_symbol_editor") (generator_version "9.0")
  (symbol "ESP32-DevKitC-V4" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)
    (property "Reference" "U" (at 0 26.67 0) {FONT})
    (property "Value" "ESP32-DevKitC-V4" (at 0 -26.67 0) {FONT})
    (property "Footprint" "garage-opener:ESP32-DevKitC-V4_Socket" (at 0 -29.21 0) {HIDDEN})
    (property "Datasheet" "https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html" (at 0 0 0) {HIDDEN})
    (property "Description" "ESP32-DevKitC V4 (ESP32-WROOM-32U) 38-pin devkit in two 1x19 sockets, rows 25.4 mm apart" (at 0 0 0) {HIDDEN})
    (symbol "ESP32-DevKitC-V4_0_1"
      (rectangle (start -12.7 25.4) (end 12.7 -25.4) (stroke (width 0.254) (type default)) (fill (type background))))
    (symbol "ESP32-DevKitC-V4_1_1"
      {chr(10).join("      " + p for p in pins).lstrip()})))
'''
(P / "lib/garage-opener.kicad_sym").write_text(sym)

def pad(n, x, y):
    shape = "rect" if n in (1, 20) else "oval"
    return (f'(pad "{n}" thru_hole {shape} (at {x} {y:.2f}) (size 1.7 1.7) (drill 1) '
            f'(layers "*.Cu" "*.Mask") (uuid "{uid("pad", n)}"))')

def line(layer, x1, y1, x2, y2, w):
    return (f'(fp_rect (start {x1} {y1}) (end {x2} {y2}) (stroke (width {w}) (type solid)) '
            f'(fill no) (layer "{layer}") (uuid "{uid(layer, x1, y1)}"))')

pads = [pad(i + 1, 0, i * 2.54) for i in range(19)] + [pad(i + 20, 25.4, i * 2.54) for i in range(19)]
fp = f'''(footprint "ESP32-DevKitC-V4_Socket" (version 20241229) (generator "pcbnew") (generator_version "9.0") (layer "F.Cu")
  (descr "Two 1x19 2.54 mm female sockets, rows 25.4 mm apart, for ESP32-DevKitC V4 (WROOM-32U). Fab outline approximate (54.4 x 27.9 mm): measure the real board.")
  (tags "ESP32 DevKitC socket")
  (property "Reference" "REF**" (at 12.7 -8 0) (layer "F.SilkS") (uuid "{uid("ref")}") (effects (font (size 1 1) (thickness 0.15))))
  (property "Value" "ESP32-DevKitC-V4_Socket" (at 12.7 52 0) (layer "F.Fab") (uuid "{uid("val")}") (effects (font (size 1 1) (thickness 0.15))))
  (attr through_hole)
  {line("F.SilkS", -1.33, -1.33, 1.33, 47.05, 0.12)}
  {line("F.SilkS", 24.07, -1.33, 26.73, 47.05, 0.12)}
  {line("F.Fab", -1.25, -6.0, 26.65, 48.4, 0.1)}
  {line("F.CrtYd", -1.8, -6.5, 27.2, 48.9, 0.05)}
  (fp_text user "ANTENNA (U.FL) END" (at 12.7 -3.5 0) (layer "F.Fab") (uuid "{uid("ant")}") (effects (font (size 1 1) (thickness 0.15))))
  (fp_text user "USB END" (at 12.7 46.5 0) (layer "F.Fab") (uuid "{uid("usb")}") (effects (font (size 1 1) (thickness 0.15))))
  {chr(10).join("  " + p for p in pads).lstrip()})
'''
(P / "lib/garage-opener.pretty/ESP32-DevKitC-V4_Socket.kicad_mod").write_text(fp)

(P / "sym-lib-table").write_text('(sym_lib_table (version 7)\n  (lib (name "garage-opener")(type "KiCad")'
    '(uri "${KIPRJMOD}/lib/garage-opener.kicad_sym")(options "")(descr "Garage opener project symbols"))\n)\n')
(P / "fp-lib-table").write_text('(fp_lib_table (version 7)\n  (lib (name "garage-opener")(type "KiCad")'
    '(uri "${KIPRJMOD}/lib/garage-opener.pretty")(options "")(descr "Garage opener project footprints"))\n)\n')
(P / "garage-opener-pcb.kicad_pro").write_text(json.dumps(
    {"meta": {"filename": "garage-opener-pcb.kicad_pro", "version": 3}}, indent=2) + "\n")
print("wrote library + skeleton to", P)
```

- [ ] **Step 4: Generate, upgrade to native KiCad 10 format, run the test**

```bash
python3 $SCRATCH/gen_lib.py hardware/garage-opener-pcb
kicad-cli sym upgrade hardware/garage-opener-pcb/lib/garage-opener.kicad_sym
kicad-cli fp upgrade hardware/garage-opener-pcb/lib/garage-opener.pretty
python3 $SCRATCH/test_lib.py hardware/garage-opener-pcb
```

Expected: `LIB OK`. If `sym upgrade` / `fp upgrade` reject the file, the error names the offending token: fix `gen_lib.py`, regenerate, retry. (`kicad-cli sym upgrade --help` / `fp upgrade --help` show the exact argument form if the above is rejected.)

- [ ] **Step 5: Look at the rendered symbol and footprint**

```bash
mkdir -p $SCRATCH/lib-svg && kicad-cli sym export svg -o $SCRATCH/lib-svg hardware/garage-opener-pcb/lib/garage-opener.kicad_sym \
  && kicad-cli fp export svg -o $SCRATCH/lib-svg hardware/garage-opener-pcb/lib/garage-opener.pretty
qlmanage -t -s 1200 -o $SCRATCH/lib-svg $SCRATCH/lib-svg/*.svg
```

Open the PNG thumbnails with the Read tool. Expected: symbol with 19 pins each side, 3V3 top-left, 5V bottom-left, GND top-right; footprint with two pad rows, square pads at pin 1 (top-left) and pin 20 (top-right).

- [ ] **Step 6: Commit**

```bash
git add hardware/garage-opener-pcb
git commit -m "feat(hw): add KiCad project skeleton with DevKitC V4 symbol and socket footprint"
```

---

### Task 2: Schematic + netlist checker

**Files:**
- Create: `hardware/garage-opener-pcb/check_schematic.py` (committed — re-run after any schematic edit)
- Create: `hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch`
- Create (scratchpad, not committed): `$SCRATCH/gen_sch.py`

**Interfaces:**
- Consumes: `garage-opener:ESP32-DevKitC-V4` (Task 1) and stock symbols `Device:{R,C,C_Polarized,D,D_Schottky,D_TVS,LED,Polyfuse}`, `Transistor_BJT:MMBT2222A` (derived from `Q_NPN_BEC`, pins 1 = B, 2 = E, 3 = C), `Relay:SANYOU_SRD_Form_C`, `Sensor:DHT11` (pins 1 VDD, 2 DATA, 3 NC, 4 GND), `Connector:Screw_Terminal_01x02/01x03`, `Mechanical:MountingHole`, `power:{+5V,+3V3,GND,PWR_FLAG}`. Device diodes and LED: pin 1 = K, pin 2 = A. `C_Polarized`: pin 1 = +.
- Produces: `check_schematic.py <project-dir>` → exit 0 and prints `SCHEMATIC OK`, else exit 1 with the failing assertion.

- [ ] **Step 1: Confirm the relay contact mapping before trusting it**

```bash
mkdir -p $SCRATCH/relay && kicad-cli sym export svg -s SANYOU_SRD_Form_C -o $SCRATCH/relay \
  /Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols/Relay.kicad_sym && qlmanage -t -s 800 -o $SCRATCH/relay $SCRATCH/relay/*.svg
```

Read the PNG. Then fetch the Songle SRD-05VDC-SL-C datasheet (WebFetch, e.g. search "SRD-05VDC-SL-C datasheet pin") and confirm the footprint pad of each number (`Relay_THT.pretty/Relay_SPDT_SANYOU_SRD_Series_Form_C.kicad_mod`) matches the datasheet's COM / NO / NC / coil pins. Expected: 1 = COM, 2 & 5 = coil, 3 = NO, 4 = NC. **If it differs, change the `K1` lines in `EXPECTED` (Step 2) and the `K1` net map in `gen_sch.py` (Step 4) to the verified numbers before continuing.**

- [ ] **Step 2: Write the failing checker**

`hardware/garage-opener-pcb/check_schematic.py`:

```python
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
func = {(n.get("ref"), n.get("pin")): n.get("pinfunction") for n in root.iter("node")}

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
```

- [ ] **Step 3: Run it to verify it fails**

Run: `python3 hardware/garage-opener-pcb/check_schematic.py`
Expected: exits non-zero with `netlist export failed` (the schematic does not exist yet).

- [ ] **Step 4: Write the schematic generator**

`$SCRATCH/gen_sch.py`:

```python
#!/usr/bin/env python3
"""Throwaway: writes hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch (rev 1) once."""
import glob, re, sys, uuid
from pathlib import Path

P = Path(sys.argv[1])
NAME = "garage-opener-pcb"
STOCK = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols")
NS = uuid.UUID("6f1c3c1e-6a43-4c1f-9a5e-3b0b8f0c2a11")
uid = lambda *k: str(uuid.uuid5(NS, "sch/" + "/".join(map(str, k))))
ROOT = uid("root")
# Use the newest schematic format version shipped with this KiCad's demos.
VERSION = max(re.search(r"\(version (\d+)\)", open(f).read()).group(1)
              for f in glob.glob("/Library/Application Support/kicad/demos/*/*.kicad_sch"))

# ---------- s-expression helpers (string-aware paren matching) ----------
def block_end(t, i):
    d, j, ins = 0, i, False
    while j < len(t):
        c = t[j]
        if ins:
            if c == "\\": j += 1
            elif c == '"': ins = False
        elif c == '"': ins = True
        elif c == "(": d += 1
        elif c == ")":
            d -= 1
            if d == 0: return j + 1
        j += 1
    raise ValueError("unbalanced")

def children(b):
    """Top-level child lists of block b (skips b's own opening paren)."""
    out, j, ins = [], 1, False
    while j < len(b) - 1:
        c = b[j]
        if ins:
            if c == "\\": j += 1
            elif c == '"': ins = False
        elif c == '"': ins = True
        elif c == "(":
            e = block_end(b, j); out.append(b[j:e]); j = e; continue
        j += 1
    return out

_libcache = {}
def lib_text(lib):
    if lib == "garage-opener":
        return (P / "lib/garage-opener.kicad_sym").read_text()
    if lib not in _libcache:
        _libcache[lib] = (STOCK / f"{lib}.kicad_sym").read_text()
    return _libcache[lib]

def raw_symbol(lib, name):
    t = lib_text(lib)
    m = re.search(r'\(symbol "' + re.escape(name) + r'"\s', t)
    if not m: raise KeyError(f"{lib}:{name}")
    return t[m.start():block_end(t, m.start())]

def flat_symbol(lib, name):
    """Library symbol with any (extends ...) resolved, as embedded in a schematic."""
    b = raw_symbol(lib, name)
    ext = re.search(r'\(extends "([^"]+)"\)', b)
    if ext:
        parent = flat_symbol(lib, ext.group(1))
        pname = ext.group(1)
        kids = children(parent)
        props = [c for c in children(b) if c.startswith("(property")]
        rest = [c for c in kids if not c.startswith("(property")]
        rest = [c.replace(f'(symbol "{pname}_', f'(symbol "{name}_') for c in rest]
        # keep parent's flags (in_bom, pin_names, ...) then child's properties, then units
        flags = [c for c in rest if not c.startswith("(symbol")]
        units = [c for c in rest if c.startswith("(symbol")]
        b = f'(symbol "{name}"\n' + "\n".join(flags + props + units) + "\n)"
    return b

def embedded(lib_id):
    lib, name = lib_id.split(":")
    b = flat_symbol(lib, name)
    return b.replace(f'(symbol "{name}"', f'(symbol "{lib_id}"', 1)

def pins_of(lib_id):
    lib, name = lib_id.split(":")
    b = flat_symbol(lib, name)
    out = {}
    for m in re.finditer(r"\(pin \w+ \w+", b):
        pb = b[m.start():block_end(b, m.start())]
        x, y, a = re.search(r"\(at ([-\d.]+) ([-\d.]+) ([-\d.]+)\)", pb).groups()
        num = re.search(r'\(number "([^"]*)"', pb).group(1)
        out[num] = (float(x), float(y), int(float(a)))
    return out

def lib_prop(lib_id, prop):
    lib, name = lib_id.split(":")
    b = flat_symbol(lib, name)
    m = re.search(r'\(property "' + prop + r'" "([^"]*)"', b)
    pb = b[m.start():block_end(b, m.start())]
    at = re.search(r"\(at ([-\d.]+) ([-\d.]+) ([-\d.]+)\)", pb).groups()
    eff = re.search(r"\(effects", pb)
    effects = pb[eff.start():block_end(pb, eff.start())]
    return m.group(1), tuple(map(float, at)), effects

# ---------- schematic items ----------
items, used = [], set()
DIR = {0: (1, 0), 90: (0, -1), 180: (-1, 0), 270: (0, 1)}   # outward direction -> schematic dx, dy
JUST = {0: "left bottom", 90: "left bottom", 180: "right bottom", 270: "right bottom"}
POWER = {"+5V", "+3V3", "GND"}
pwr_n = [0]

def fmt(v): return f"{v:.2f}".rstrip("0").rstrip(".")

def symbol(ref, lib_id, x, y, value=None, footprint=None, fields=None, dnp=False, rot=0, hide_ref=False):
    used.add(lib_id)
    props = []
    for i, prop in enumerate(("Reference", "Value", "Footprint", "Datasheet")):
        libval, (px, py, pr), eff = lib_prop(lib_id, prop)
        val = {"Reference": ref, "Value": value or libval,
               "Footprint": footprint if footprint is not None else libval}.get(prop, libval)
        hide = prop in ("Footprint", "Datasheet") or (prop == "Reference" and hide_ref)
        props.append(f'(property "{prop}" "{val}" (at {fmt(x+px)} {fmt(y-py)} {fmt(pr)})'
                     f'{" (hide yes)" if hide else ""} {eff})')
    for k, v in (fields or {}).items():
        props.append(f'(property "{k}" "{v}" (at {fmt(x)} {fmt(y)} 0) (hide yes) (effects (font (size 1.27 1.27))))')
    pins = " ".join(f'(pin "{n}" (uuid "{uid(ref, "pin", n)}"))' for n in pins_of(lib_id))
    items.append(
        f'(symbol (lib_id "{lib_id}") (at {fmt(x)} {fmt(y)} {rot}) (unit 1) (exclude_from_sim no) '
        f'(in_bom {"no" if ref.startswith("#") or "(in_bom no)" in flat_symbol(*lib_id.split(":")) else "yes"}) '
        f'(on_board {"no" if ref.startswith("#") else "yes"}) '
        f'(dnp {"yes" if dnp else "no"}) (uuid "{uid(ref)}")\n  ' + "\n  ".join(props) + f"\n  {pins}\n"
        f'  (instances (project "{NAME}" (path "/{ROOT}" (reference "{ref}") (unit 1)))))')

def wire(x1, y1, x2, y2):
    items.append(f'(wire (pts (xy {fmt(x1)} {fmt(y1)}) (xy {fmt(x2)} {fmt(y2)})) '
                 f'(stroke (width 0) (type default)) (uuid "{uid("w", x1, y1, x2, y2)}"))')

def power_symbol(net, x, y, d):
    pwr_n[0] += 1
    base = 270 if net == "GND" else 90           # GND drawn pointing down, rails pointing up
    symbol(f"#PWR{pwr_n[0]:02d}", f"power:{net}", x, y, value=net, footprint="", rot=(d - base) % 360,
           hide_ref=True)

def label(net, x, y, d):
    items.append(f'(label "{net}" (at {fmt(x)} {fmt(y)} {d}) (fields_autoplaced yes) '
                 f'(effects (font (size 1.27 1.27)) (justify {JUST[d]})) (uuid "{uid("l", net, x, y)}"))')

def no_connect(x, y):
    items.append(f'(no_connect (at {fmt(x)} {fmt(y)}) (uuid "{uid("nc", x, y)}"))')

def part(ref, lib_id, x, y, nets, **kw):
    """Place a symbol (rotation 0) and connect each pin: net name, power net, or 'NC'."""
    symbol(ref, lib_id, x, y, **kw)
    pins = pins_of(lib_id)
    assert set(nets) <= set(pins), f"{ref}: unknown pins {set(nets) - set(pins)}"
    for num, net in nets.items():
        px, py, a = pins[num]
        ex, ey = x + px, y - py                       # library y is up, schematic y is down
        if net == "NC":
            no_connect(ex, ey); continue
        d = (a + 180) % 360
        dx, dy = DIR[d]
        sx, sy = ex + dx * 2.54, ey + dy * 2.54
        wire(ex, ey, sx, sy)
        (power_symbol if net in POWER else label)(net, sx, sy, d)

def text(s, x, y, size=1.27):
    s = s.replace('"', "'").replace("\n", "\\n")
    items.append(f'(text "{s}" (exclude_from_sim no) (at {fmt(x)} {fmt(y)} 0) '
                 f'(effects (font (size {size} {size})) (justify left top)) (uuid "{uid("t", x, y)}"))')

R0805, C0805 = "Resistor_SMD:R_0805_2012Metric", "Capacitor_SMD:C_0805_2012Metric"
TB2 = "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal"
TB3 = "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-3_1x03_P5.00mm_Horizontal"
LCSC = lambda c: {"LCSC": c} if c else {}

# ---------- U1: devkit ----------
text("ESP32 DevKitC V4 (WROOM-32U) - plugs into J4/J5 sockets", 30.48, 88.9, 1.75)
devkit = {"1": "+3V3", "14": "GND", "19": "+5V", "20": "GND", "26": "GND",
          "7": "BOARD_ID0", "8": "BOARD_ID1", "22": "DHT_DATA", "29": "RELAY_DRV",
          "31": "LED_DOOR", "32": "REED", "34": "LED_RUN"}
devkit.update({str(p): "NC" for p in range(1, 39) if str(p) not in devkit})
part("U1", "garage-opener:ESP32-DevKitC-V4", 63.5, 127, devkit)

# ---------- power ----------
text("POWER: 5 V in -> PTC fuse -> Schottky (reverse-polarity + USB back-feed block) -> +5V", 119.38, 30.48, 1.75)
part("J1", "Connector:Screw_Terminal_01x02", 127, 48.26, {"1": "VIN_RAW", "2": "GND"},
     value="5V IN", footprint=TB2)
part("F1", "Device:Polyfuse", 152.4, 48.26, {"1": "VIN_RAW", "2": "VIN_FUSED"},
     value="0.5A hold", footprint="Fuse:Fuse_1812_4532Metric")
part("D1", "Device:D_Schottky", 177.8, 48.26, {"2": "VIN_FUSED", "1": "+5V"},
     value="SS34", footprint="Diode_SMD:D_SMA", fields=LCSC("C8678"))
part("C1", "Device:C_Polarized", 203.2, 48.26, {"1": "+5V", "2": "GND"},
     value="100uF 16V", footprint="Capacitor_THT:CP_Radial_D6.3mm_P2.50mm")
part("C2", "Device:C", 220.98, 48.26, {"1": "+5V", "2": "GND"},
     value="100nF", footprint=C0805, fields=LCSC("C49678"))
part("#FLG01", "power:PWR_FLAG", 238.76, 45.72, {"1": "+5V"}, footprint="", hide_ref=True)
part("#FLG02", "power:PWR_FLAG", 254.0, 45.72, {"1": "GND"}, footprint="", hide_ref=True)

# ---------- relay ----------
text("RELAY (GPIO5): R2 4k7 pull-down holds GPIO5 ~0.3 V during boot (internal ~45k pull-up),\n"
     "below Q1 Vbe -> relay cannot pull in while the ESP32 resets. J2: wire COM+NO across the opener's wall button.",
     119.38, 81.28, 1.5)
part("R1", "Device:R", 127, 106.68, {"1": "RELAY_DRV", "2": "Q1_BASE"}, value="1k", footprint=R0805, fields=LCSC("C17513"))
part("R2", "Device:R", 139.7, 106.68, {"1": "RELAY_DRV", "2": "GND"}, value="4k7", footprint=R0805, fields=LCSC("C17673"))
part("Q1", "Transistor_BJT:MMBT2222A", 157.48, 106.68, {"1": "Q1_BASE", "2": "GND", "3": "COIL_LO"})
part("K1", "Relay:SANYOU_SRD_Form_C", 190.5, 106.68,
     {"5": "+5V", "2": "COIL_LO", "1": "RELAY_COM", "3": "RELAY_NO", "4": "RELAY_NC"},
     value="SRD-05VDC-SL-C", fields=LCSC("C35449"))
part("D2", "Device:D", 220.98, 106.68, {"1": "+5V", "2": "COIL_LO"},
     value="1N4148W", footprint="Diode_SMD:D_SOD-123", fields=LCSC("C81598"))
part("J2", "Connector:Screw_Terminal_01x03", 243.84, 106.68,
     {"1": "RELAY_COM", "2": "RELAY_NO", "3": "RELAY_NC"}, value="OPENER COM/NO/NC", footprint=TB3)

# ---------- reed ----------
text("REED SWITCH (GPIO4): closed = low, open = high. 10k pull-up + 1k/100nF filter (~1 ms) + ESD at J3.",
     119.38, 137.16, 1.5)
part("J3", "Connector:Screw_Terminal_01x02", 127, 154.94, {"1": "REED_EXT", "2": "GND"},
     value="REED", footprint=TB2)
part("D3", "Device:D_TVS", 149.86, 154.94, {"1": "REED_EXT", "2": "GND"},
     value="PESD5V0S1BA", footprint="Diode_SMD:D_SOD-323")
part("R4", "Device:R", 172.72, 154.94, {"1": "REED_EXT", "2": "REED"}, value="1k", footprint=R0805, fields=LCSC("C17513"))
part("R3", "Device:R", 190.5, 154.94, {"1": "+3V3", "2": "REED"}, value="10k", footprint=R0805, fields=LCSC("C17414"))
part("C3", "Device:C", 208.28, 154.94, {"1": "REED", "2": "GND"}, value="100nF", footprint=C0805, fields=LCSC("C49678"))

# ---------- DHT22 ----------
text("DHT22 / AM2302 (GPIO22), on board. Place at the far end from U1/K1 on a slotted neck; vent the enclosure.",
     119.38, 182.88, 1.5)
part("U2", "Sensor:DHT11", 132.08, 203.2, {"1": "+3V3", "2": "DHT_DATA", "3": "NC", "4": "GND"},
     value="DHT22", footprint="Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical")
part("R5", "Device:R", 160.02, 203.2, {"1": "+3V3", "2": "DHT_DATA"}, value="4k7", footprint=R0805, fields=LCSC("C17673"))
part("C4", "Device:C", 177.8, 203.2, {"1": "+3V3", "2": "GND"}, value="100nF", footprint=C0805, fields=LCSC("C49678"))

# ---------- LEDs ----------
text("LEDs: RUN (GPIO2, strapping pin - LED+R to GND is a safe load) and DOOR (GPIO16). ~4 mA each.",
     119.38, 228.6, 1.5)
part("R6", "Device:R", 127, 246.38, {"1": "LED_RUN", "2": "LED_RUN_A"}, value="330", footprint=R0805, fields=LCSC("C17630"))
part("D4", "Device:LED", 149.86, 246.38, {"2": "LED_RUN_A", "1": "GND"}, value="RUN green", footprint="LED_THT:LED_D3.0mm")
part("R7", "Device:R", 177.8, 246.38, {"1": "LED_DOOR", "2": "LED_DOOR_A"}, value="330", footprint=R0805, fields=LCSC("C17630"))
part("D5", "Device:LED", 200.66, 246.38, {"2": "LED_DOOR_A", "1": "GND"}, value="DOOR amber", footprint="LED_THT:LED_D3.0mm")

# ---------- board ID ----------
text("BOARD ID (GPIO32 = ID0, GPIO33 = ID1), not read by firmware yet.\nFit exactly one resistor per line. "
     "Rev 1 = 00: R9, R11 fitted; R8, R10 DNP.\nPrototype (pins floating, internal pull-ups) reads 11.",
     271.78, 137.16, 1.5)
part("R8", "Device:R", 281.94, 160.02, {"1": "+3V3", "2": "BOARD_ID0"}, value="10k", footprint=R0805, dnp=True, fields=LCSC("C17414"))
part("R9", "Device:R", 299.72, 160.02, {"1": "BOARD_ID0", "2": "GND"}, value="10k", footprint=R0805, fields=LCSC("C17414"))
part("R10", "Device:R", 317.5, 160.02, {"1": "+3V3", "2": "BOARD_ID1"}, value="10k", footprint=R0805, dnp=True, fields=LCSC("C17414"))
part("R11", "Device:R", 335.28, 160.02, {"1": "BOARD_ID1", "2": "GND"}, value="10k", footprint=R0805, fields=LCSC("C17414"))

# ---------- mechanical + notes ----------
text("MOUNTING: 4x M3", 271.78, 182.88, 1.5)
for i in range(4):
    part(f"H{i+1}", "Mechanical:MountingHole", 281.94 + i * 15.24, 195.58, {},
         value="M3", footprint="MountingHole:MountingHole_3.2mm_M3")
text("NOTES\n"
     "1. Firmware unchanged: GPIO map = main/board.h. Reset/factory button = devkit Boot button (GPIO0).\n"
     "2. External 5 V (J1) and USB together only if the devkit has a diode on USB VBUS - check first.\n"
     "3. Devkit rows are 25.4 mm apart (DevKitC V4) - measure the real board before ordering PCBs.\n"
     "4. Relay output is for the opener's low-voltage push-button input. Not for mains.\n"
     "5. Bring-up checklist: docs/superpowers/specs/2026-10-16-carrier-pcb-schematic-design.md",
     271.78, 30.48, 1.5)

lib_symbols = "\n".join(embedded(l) for l in sorted(used))
sch = f'''(kicad_sch (version {VERSION}) (generator "eeschema") (generator_version "10.0")
  (uuid "{ROOT}")
  (paper "A3")
  (title_block (title "Garage opener carrier PCB") (date "2026-10-16") (rev "1")
    (comment 1 "ESP32 DevKitC V4 (WROOM-32U) carrier - GPIO map = main/board.h")
    (comment 2 "Lars-Ake Ekstrand"))
  (lib_symbols
{lib_symbols}
  )
{chr(10).join(items)}
  (sheet_instances (path "/" (page "1")))
  (embedded_fonts no)
)
'''
(P / f"{NAME}.kicad_sch").write_text(sch)
print(f"wrote {NAME}.kicad_sch: {len(items)} items, {len(used)} lib symbols, format {VERSION}")
```

- [ ] **Step 5: Generate, upgrade to native format, and run ERC**

```bash
python3 $SCRATCH/gen_sch.py hardware/garage-opener-pcb
kicad-cli sch upgrade hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch
kicad-cli sch erc --severity-all --format report -o $SCRATCH/erc.rpt hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch; cat $SCRATCH/erc.rpt
```

Expected: `upgrade` succeeds (a load error names the bad token → fix `gen_sch.py`, regenerate). ERC report shows **0 errors**. Warnings: `lib_symbol_mismatch` on `MMBT2222A` (flattened derived symbol) is acceptable; any other warning gets fixed in `gen_sch.py` (e.g. off-grid → round coordinates to 1.27) or explained in the final report.

- [ ] **Step 6: Run the checker**

Run: `python3 hardware/garage-opener-pcb/check_schematic.py`
Expected: `SCHEMATIC OK`. If only the DNP checks fail, inspect how this KiCad marks DNP in the XML (`grep -n -i dnp` on an exported netlist) and adjust the `dnp = ...` line in the checker to match — then confirm R8/R10 really are DNP in the schematic. On other `FAIL` lines, fix the generator (not the expectations — except the relay mapping, which Step 1 decides), regenerate, repeat Steps 5–6.

- [ ] **Step 7: Prove the checker catches a firmware mismatch**

```bash
sed -i '' 's/DOOR_RELAY_GPIO   GPIO_NUM_5/DOOR_RELAY_GPIO   GPIO_NUM_26/' main/board.h
python3 hardware/garage-opener-pcb/check_schematic.py; echo "exit=$?"
git checkout main/board.h
```

Expected: `FAIL: DOOR_RELAY_GPIO=GPIO26: ...` and `exit=1`; then `git status --short main/board.h` prints nothing.

- [ ] **Step 8: Look at the schematic**

```bash
kicad-cli sch export pdf -o $SCRATCH/preview.pdf hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch
```

Read `$SCRATCH/preview.pdf`. Expected: all blocks legible, no overlapping labels/parts, text notes readable. Fix positions in `gen_sch.py` if anything overlaps, regenerate, re-run Steps 5–6.

- [ ] **Step 9: Commit**

```bash
git add hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch hardware/garage-opener-pcb/check_schematic.py
git commit -m "feat(hw): add rev 1 carrier-PCB schematic and netlist checker"
```

---

### Task 3: Verified BOM, PDF export, and docs

**Files:**
- Modify: `hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch` (only if an LCSC number fails verification)
- Create: `hardware/garage-opener-pcb/bom.csv`
- Create: `hardware/garage-opener-pcb/README.md`
- Replace: `docs/Schematic_GarageOpener.pdf`
- Modify: `README.md:14`
- Modify: `docs/wiring-wroom-32u.html:316`

**Interfaces:**
- Consumes: the schematic and `check_schematic.py` from Task 2.

- [ ] **Step 1: Verify every LCSC number**

For each of C8678 (SS34), C49678 (100 nF 0805), C17513 (1 kΩ 0805), C17673 (4.7 kΩ 0805), C35449 (SRD-05VDC-SL-C), C81598 (1N4148W SOD-123), C17414 (10 kΩ 0805), C17630 (330 Ω 0805): WebFetch `https://www.lcsc.com/product-detail/<C-number>.html` (or search the number) and confirm value + package. For any mismatch, set the `LCSC` field of those symbols to the correct number, or to empty if none is confirmed — edit the `.kicad_sch` directly (`(property "LCSC" "C…"` lines). Do not invent numbers for parts left without one.

- [ ] **Step 2: Export the BOM and check DNP exclusion**

```bash
cd hardware/garage-opener-pcb
kicad-cli sch export bom --help | grep -E -- '--(fields|labels|group-by|exclude-dnp)'
kicad-cli sch export bom -o bom.csv --fields 'Reference,Value,Footprint,LCSC,${QUANTITY}' \
  --labels 'Refs,Value,Footprint,LCSC,Qty' --group-by 'Value,Footprint,LCSC' --exclude-dnp garage-opener-pcb.kicad_sch
cat bom.csv
python3 -c "import csv,re,sys; refs=[r for row in csv.DictReader(open('bom.csv')) for r in re.split(r'[ ,]+',row['Refs']) if r]; \
  bad={'R8','R10'}&set(refs); assert not bad, bad; assert not [r for r in refs if r.startswith('#')]; print('BOM OK', len(refs), 'parts')"
cd -
```

Expected: `BOM OK 26 parts` (U1, U2, J1–J3, F1, D1–D5, Q1, K1, C1–C4, R1–R7, R9, R11 = 26; H1–H4 are excluded because the stock `MountingHole` symbol is `in_bom no`). Flag names were confirmed against KiCad 10.0.6 `--help`.

- [ ] **Step 3: Export the PDF over the old one and look at it**

```bash
kicad-cli sch export pdf -o docs/Schematic_GarageOpener.pdf hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch
```

Read `docs/Schematic_GarageOpener.pdf`. Expected: rev 1 KiCad schematic, title block "Garage opener carrier PCB", all blocks readable.

- [ ] **Step 4: Write `hardware/garage-opener-pcb/README.md`**

```markdown
# Garage opener carrier PCB (rev 1)

KiCad 10 project for a carrier board that an **ESP32 DevKitC V4 (ESP32-WROOM-32U, 38-pin)** plugs
into. It adds a 5 V screw-terminal input (PTC fuse + Schottky), an on-board SRD-05VDC relay for the
opener's push-button input, a protected reed-switch input, an on-board DHT22, RUN/DOOR LEDs and two
board-ID straps. The GPIO map is identical to the prototype (`main/board.h`), so the same firmware
and OTA image run on both.

- Schematic: `garage-opener-pcb.kicad_sch` (PDF: `../../docs/Schematic_GarageOpener.pdf`)
- BOM: `bom.csv` (DNP parts excluded)
- Design + bring-up checklist: `../../docs/superpowers/specs/2026-10-16-carrier-pcb-schematic-design.md`

## Checking the schematic after edits

    python3 check_schematic.py

Exports the netlist with `kicad-cli` and checks every net, footprint, the DNP ID straps, and that
each GPIO in `main/board.h` lands on the right circuit. Run ERC too:

    kicad-cli sch erc --severity-all garage-opener-pcb.kicad_sch

## Before ordering a PCB

- Measure the devkit: rows must be 25.4 mm apart (footprint `garage-opener:ESP32-DevKitC-V4_Socket`).
- Check whether the devkit has a diode on USB VBUS before using USB and J1 at the same time.
- Board ID rev 1 = `00` (R9, R11 fitted). Future revisions: fit R8 and/or R10 instead.
```

- [ ] **Step 5: Update the top-level README and wiring doc**

`README.md` line 14 — replace the whole line with:

```markdown
Schematics for the hardware are in [docs/Schematic_GarageOpener.pdf](docs/Schematic_GarageOpener.pdf), drawn in [KiCad](https://www.kicad.org/) — the project (rev 1 carrier PCB for an ESP32 DevKitC V4) is in [hardware/garage-opener-pcb](hardware/garage-opener-pcb).
```

`docs/wiring-wroom-32u.html` line 316 — in the note, replace
`Old board pulls the relay EN to VDD via 20 kΩ (R3) — keep it if your carrier has it.`
with
`The old prototype has a 20 kΩ (R3) from GPIO5 to GND. The rev 1 PCB (hardware/garage-opener-pcb) drives the relay through a transistor with a 4.7 kΩ pull-down so it cannot click during boot.`

- [ ] **Step 6: Final verification**

```bash
python3 hardware/garage-opener-pcb/check_schematic.py
kicad-cli sch erc --severity-error --exit-code-violations hardware/garage-opener-pcb/garage-opener-pcb.kicad_sch; echo "erc exit=$?"
git status --short
```

Expected: `SCHEMATIC OK`, `erc exit=0`, and only the files listed in this task modified/added. `git diff --stat main/` is empty (firmware untouched).

- [ ] **Step 7: Commit**

```bash
git add hardware/garage-opener-pcb/bom.csv hardware/garage-opener-pcb/README.md docs/Schematic_GarageOpener.pdf README.md docs/wiring-wroom-32u.html
git commit -m "docs(hw): export rev 1 schematic PDF and BOM, point docs at the KiCad project"
```
