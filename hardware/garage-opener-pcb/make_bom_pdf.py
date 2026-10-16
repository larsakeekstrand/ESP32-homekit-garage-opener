#!/usr/bin/env python3
"""Renders bom.csv as docs/BOM_GarageOpener.pdf (via headless Google Chrome).

Usage: python3 make_bom_pdf.py   (re-export bom.csv from KiCad first if the schematic changed)
"""
import csv, html, subprocess, sys, tempfile
from datetime import date
from pathlib import Path

PROJ = Path(__file__).resolve().parent
OUT = PROJ.parent.parent / "docs/BOM_GarageOpener.pdf"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"

# Not in bom.csv: DNP parts, and items that have no schematic symbol of their own.
DNP = [("R8, R10", "10k", "R_0805_2012Metric", "C17414", "Board-ID straps to +3V3 — fit only on future revisions")]
EXTRA = [
    ("1", "ESP32 DevKitC V4 (ESP32-WROOM-32U)", "Plugs into U1. Needs an external 2.4 GHz antenna on its U.FL connector."),
    ("2", "Female header 1x19, 2.54 mm", "U1 sockets (rows 25.4 mm apart)"),
    ("1", "Plug Phoenix MC 1,5/2-ST-3,81 (or 15EDGK clone)", "Mates with J1 (5 V in)"),
    ("1", "Plug Phoenix MSTB 2,5/2-ST-5,08 (or 2EDGK clone)", "Mates with J3 (reed)"),
    ("1", "Plug Phoenix MSTB 2,5/3-ST-5,08 (or 2EDGK clone)", "Mates with J2 (relay COM/NO/NC)"),
    ("1", "U.FL (IPEX1) to SMA-female bulkhead pigtail, ~10 cm + 2.4 GHz SMA antenna", "Antenna through the enclosure wall"),
    ("4", "M3 heat-set insert, M3×5×4 mm", "Enclosure base bosses under H1–H4"),
    ("4", "M3×25 countersunk screw", "Lid: through the lid tubes and H1–H4 into the inserts, clamps the PCB"),
    ("2", "4 mm pan-head wood screw (head ≤ Ø8 × 3.2 mm) + wall plug", "Wall: the enclosure hangs on them by its keyholes"),
    ("1", "5.1–5.2 V DC power supply", "Into J1. A plain 5.0 V supply leaves too little margin."),
]

rows = list(csv.DictReader(open(PROJ / "bom.csv")))
total = sum(int(r["Qty"]) for r in rows)
e = html.escape

def fp(s):
    return e(s.split(":", 1)[-1])

body = "".join(
    f"<tr><td>{e(r['Refs'])}</td><td class=n>{e(r['Qty'])}</td><td>{e(r['Value'])}</td>"
    f"<td class=fp>{fp(r['Footprint'])}</td><td class=m>{e(r['LCSC']) or '—'}</td></tr>"
    for r in rows)
dnp = "".join(f"<tr><td>{e(a)}</td><td>{e(b)}</td><td class=fp>{e(c)}</td><td class=m>{e(d)}</td><td>{e(n)}</td></tr>"
              for a, b, c, d, n in DNP)
extra = "".join(f"<tr><td class=n>{e(q)}</td><td>{e(i)}</td><td>{e(n)}</td></tr>" for q, i, n in EXTRA)

page = f"""<!doctype html><html><head><meta charset="utf-8"><title>BOM — Garage opener carrier PCB</title>
<style>
@page {{ size: A4 portrait; margin: 12mm; }}
html, body {{ background: #fff; -webkit-print-color-adjust: exact; print-color-adjust: exact; }}
body {{ font: 8.8pt -apple-system, Helvetica, Arial, sans-serif; color: #16191d; margin: 0; }}
h1 {{ font-size: 16pt; margin: 0 0 2mm; }}
h2 {{ font-size: 11pt; margin: 3.5mm 0 1mm; }}
.meta {{ color: #5b6470; margin-bottom: 4mm; }}
table {{ width: 100%; border-collapse: collapse; }}
th {{ text-align: left; font-size: 8.5pt; text-transform: uppercase; letter-spacing: .04em; color: #5b6470;
      border-bottom: 1.5px solid #c4cad2; padding: 1.5mm 2mm; }}
td {{ border-bottom: 1px solid #eaedf1; padding: 0.6mm 2mm; vertical-align: top; }}
td.n {{ text-align: right; width: 10mm; }}
td.fp, td.m {{ font: 8.5pt ui-monospace, Menlo, monospace; }}
tr {{ break-inside: avoid; }}
.foot {{ margin-top: 4mm; font-size: 8.5pt; color: #5b6470; }}
</style></head><body>
<h1>Garage opener carrier PCB — Bill of Materials</h1>
<div class=meta>Rev 1 · {total} fitted parts · generated {date.today().isoformat()} from
hardware/garage-opener-pcb/bom.csv · schematic: docs/Schematic_GarageOpener.pdf</div>
<table><thead><tr><th>Refs</th><th>Qty</th><th>Value</th><th>Footprint</th><th>LCSC</th></tr></thead>
<tbody>{body}</tbody></table>
<h2>Not populated (DNP)</h2>
<table><thead><tr><th>Refs</th><th>Value</th><th>Footprint</th><th>LCSC</th><th>Note</th></tr></thead>
<tbody>{dnp}</tbody></table>
<h2>Also needed (no schematic line)</h2>
<table><thead><tr><th>Qty</th><th>Item</th><th>Note</th></tr></thead><tbody>{extra}</tbody></table>
<div class=foot>LCSC numbers were verified on lcsc.com when the BOM was created; “—” means no number has been chosen yet.
Before ordering a PCB, read the checklist in hardware/garage-opener-pcb/README.md.</div>
</body></html>"""

with tempfile.TemporaryDirectory() as t:
    src = Path(t) / "bom.html"
    src.write_text(page)
    r = subprocess.run([CHROME, "--headless", "--disable-gpu", "--no-pdf-header-footer",
                        f"--print-to-pdf={OUT}", src.as_uri()], capture_output=True, text=True)
    if r.returncode != 0 or not OUT.is_file():
        sys.exit(f"Chrome PDF export failed:\n{r.stderr}")
print(f"wrote {OUT.relative_to(PROJ.parent.parent)} ({total} fitted parts)")
