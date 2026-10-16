# Design: Carrier-PCB schematic (rev 1) for the garage opener

**Date:** 2026-10-16
**Status:** Approved (brainstorming), pending implementation plan
**Target:** New KiCad schematic replacing `docs/Schematic_GarageOpener.pdf` (EasyEDA, 2021)
**Firmware:** unchanged — the PCB uses the exact GPIO map in `main/board.h`

## Overview

Replace the old hand-drawn EasyEDA schematic with a correct, ERC-clean KiCad schematic for a
**carrier PCB**: an ESP32 DevKitC V4 (ESP32-WROOM-32U, 38-pin) plugs into female headers, and the
PCB carries the relay, reed-switch input, an on-board DHT22, two LEDs, 5 V screw-terminal power, and
two board-ID straps for future firmware use. Every symbol has a footprint so the schematic can go
straight to PCB layout later.

## Goals

- A KiCad project whose schematic passes ERC with zero errors and whose every symbol has a
  footprint from KiCad's stock libraries (or a project-local one where none exists).
- Exactly the same GPIO assignments as the current prototype, so one `garage.bin` (the single OTA
  asset every device pulls) runs on both the prototype and this board.
- The relay **cannot** pull in while the ESP32 boots or resets (fixes the GPIO5 strapping hazard
  in hardware).
- Reserve two board-ID pins so a future firmware can tell boards apart at runtime.
- A PDF export of the new schematic replaces `docs/Schematic_GarageOpener.pdf`.

## Non-goals

- No PCB layout, Gerbers, or ordering in this task — layout guidance only (see below).
- No firmware change. The board-ID pins are not read by any firmware yet.
- No new enclosure design (the existing STLs are for the old prototype).
- No bare-module design (own regulator / USB-UART) — the devkit supplies those.
- Not for switching mains: the relay output is for the opener's low-voltage push-button input.

## Background: problems in the old schematic

- LED series resistors were 100 Ω (≈13 mA from 3.3 V, harsh on the GPIOs); 330 Ω is correct.
- R3 (20 kΩ) sits between the relay carrier EN and GND, while `docs/wiring-wroom-32u.html`
  describes it as a pull-up to VDD — the documentation and the drawing disagree.
- No connectors for the reed switch, DHT22, power, or relay output; no input protection on the
  long reed-switch cable; no decoupling.
- Ambiguous power wiring between `Vin` and `3v3`.

## GPIO map (authoritative source: `main/board.h`)

| Function | GPIO | DevKitC pin | Notes |
|---|---|---|---|
| Door relay | GPIO5 | right row, pin 10 | Strapping pin — see boot-glitch fix |
| Reed door sensor | GPIO4 | right 13 | Closed = low, open = high (unchanged polarity) |
| DHT22 data | GPIO22 | right 3 | |
| Running LED | GPIO2 | right 15 | Strapping pin; LED + resistor to GND is a safe load |
| Door-state LED | GPIO16 | right 12 | Free on WROOM-32U (no PSRAM) |
| Reset / factory | GPIO0 | right 14 | Devkit's own Boot button; nothing added on the PCB |
| Board ID 0 | GPIO32 | left row, pin 7 | New, unused by firmware |
| Board ID 1 | GPIO33 | left 8 | New, unused by firmware |

All other devkit pins are no-connect on the PCB.

## Architecture

### Devkit symbol and footprint (U1)

A project-local symbol `ESP32-DevKitC-V4` with all 38 pins named as on the board silk:

- Left row (U1 pins 1–19, top to bottom with USB at the bottom): 3V3, EN, VP, VN, IO34, IO35, IO32, IO33, IO25, IO26, IO27, IO14, IO12,
  GND, IO13, SD2, SD3, CMD, 5V.
- Right row (U1 pins 20–38, top to bottom): GND, IO23, IO22, TX, RX, IO21, GND, IO19, IO18, IO5, IO17, IO16, IO4,
  IO0, IO2, IO15, SD1, SD0, CLK.

A project-local footprint holds two 1×19 2.54 mm female-socket rows **25.4 mm (1.0″) apart**
(the DevKitC V4 spacing). One footprint with both rows keeps the spacing fixed during layout.
**Before ordering a PCB, measure the actual board** with calipers — clones occasionally differ.
The 3V3 pin is a power *output* (devkit regulator) and 5V is a power *input* (from this PCB).

### Power

- **J1** — 2-pin 5.0 mm screw terminal, "+5V IN" / "GND".
- **F1** — 0.5 A hold resettable PTC fuse, 1812.
- **D1** — SS34 Schottky (SMA) in series: reverse-polarity protection, and stops USB VBUS from
  back-feeding the external supply. Output net **+5V** (≈4.6–4.7 V after the diode drop; enough for the
  devkit's AMS1117-class LDO and the relay's ≥75 % pull-in voltage).
- **C1** 100 µF / 16 V electrolytic (radial 6.3 mm) + **C2** 100 nF on +5V.
- **+5V** feeds U1 5V pin and the relay coil.
- **+3V3** is taken from U1's 3V3 pin and feeds only the DHT22, pull-ups and ID straps (a few mA).
- Caveat (documented on the schematic): the external 5 V and USB may both be connected only if the
  devkit has a diode on its USB VBUS. Check this before connecting both at once.

### Relay driver (GPIO5)

- **R1** 1 kΩ base resistor, GPIO5 → **Q1** MMBT2222A (SOT-23) base.
- **R2** 4.7 kΩ pull-down, GPIO5 → GND. **Boot-glitch fix:** during reset/boot GPIO5 is held high only
  by its internal ≈45 kΩ pull-up until `door_control_init()` drives it low. With R2 the pin sits at
  ≈0.3 V, below the ≈0.6 V a BJT base needs, so the relay stays off. (GPIO5 low at boot only alters
  SDIO-slave timing, which is unused.) When driven high: ≈2.5 mA base current, forced β ≈ 29 for the
  ≈71 mA coil — well saturated.
- **K1** SRD-05VDC-SL-C (Songle/Sanyou SRD form C, 5 V coil, 70 Ω). KiCad stock symbol/footprint
  `Relay_SPDT_SANYOU_SRD_Series_Form_C`.
- **D2** 1N4148W (SOD-123) flyback across the coil.
- **J2** — 3-pin 5.0 mm screw terminal: COM / NO / NC. Wire COM + NO in parallel with the opener's
  wall button; NC is spare.

### Reed-switch input (GPIO4)

- **J3** — 2-pin 5.0 mm screw terminal: REED / GND. The reed switch connects between them.
- **D3** PESD5V0S1BA (SOD-323) ESD diode at the connector, REED → GND.
- **R4** 1 kΩ series from the connector to GPIO4.
- **R3** 10 kΩ pull-up on the GPIO side to +3V3 (stiffer than the internal pull-up for a long cable;
  the firmware's internal pull-up stays enabled and is harmless in parallel).
- **C3** 100 nF GPIO4 → GND. RC ≈ 1 ms with R3, far below the firmware's 50 ms debounce.
- Polarity unchanged: closed → low, open → high.

### DHT22 (GPIO22), on board

- **U2** DHT22/AM2302, 4-pin THT at 2.54 mm: 1 VCC (+3V3), 2 DATA, 3 NC, 4 GND. Footprint:
  1×04 2.54 mm pin header/socket (the stock DHT11 footprint has the wrong body outline).
- **R5** 4.7 kΩ DATA pull-up to +3V3; **C4** 100 nF VCC → GND next to the sensor.

### LEDs

- **D4** green 3 mm THT "RUN" (GPIO2): GPIO2 → **R6** 330 Ω → anode, cathode → GND (≈4 mA).
- **D5** amber 3 mm THT "DOOR" (GPIO16): GPIO16 → **R7** 330 Ω → anode, cathode → GND.

### Board ID straps (GPIO32, GPIO33)

Each ID line has two 10 kΩ 0805 pads; fit exactly one:

| Line | To +3V3 | To GND | Rev 1 fit | Rev 1 value |
|---|---|---|---|---|
| ID0 (GPIO32) | R8 (DNP) | R9 (fitted) | R9 | 0 |
| ID1 (GPIO33) | R10 (DNP) | R11 (fitted) | R11 | 0 |

Rev 1 reads `00`. The prototype leaves these pins floating, so with internal pull-ups enabled it
reads `11`. `01` and `10` are free for future revisions. DNP parts carry the KiCad "Do not populate"
attribute so they are excluded from the BOM.

### Mechanical

- **H1–H4** M3 mounting holes (`MountingHole_3.2mm_M3`).

### Common parts

Passives 0805 SMD (hand-solderable). Screw terminals Phoenix MKDS-1,5 style 5.0 mm pitch. Each BOM
line carries value, footprint and an LCSC part number where one is readily available.

## Layout guidance (for the later PCB task; not implemented now)

- Put U2 (DHT22) at the far end from U1 and K1, on a narrow neck with routed slots around it;
  the enclosure needs vents beside it. Expect Wi-Fi and coil self-heating to bias temperature
  upward otherwise.
- Keep the U.FL antenna end of the devkit at a board edge, with no copper underneath.
- Relay contact traces ≥1 mm wide; keep contact nets ≥2 mm from logic nets.
- D3 close to J3; C2 and C4 close to their loads.

## Repository layout

- `hardware/garage-opener-pcb/garage-opener-pcb.kicad_pro`, `.kicad_sch`
- `hardware/garage-opener-pcb/lib/` — project-local symbol (`garage-opener.kicad_sym`) and
  footprint library (`garage-opener.pretty/`), with `sym-lib-table` / `fp-lib-table`
- `hardware/garage-opener-pcb/bom.csv` — exported BOM
- `docs/Schematic_GarageOpener.pdf` — replaced with the KiCad PDF export
- `README.md` and `docs/wiring-wroom-32u.html` — point to the new schematic; correct the R3 text

The schematic is produced once by a throwaway generator script (kept outside the repo); after that
the `.kicad_sch` is the source of truth, edited in KiCad.

## Tooling

KiCad installed with `brew install --cask kicad` (provides `kicad-cli` and the stock libraries).

## Verification

1. `kicad-cli sch erc` — zero errors (warnings reviewed and either fixed or explained).
2. Every symbol has a footprint that exists in the stock or project library.
3. Every GPIO net checked against `main/board.h` (table above) — no firmware change needed.
4. `kicad-cli sch export pdf` produces the new `docs/Schematic_GarageOpener.pdf`; visually reviewed.
5. `kicad-cli sch export bom` produces `bom.csv`, excluding DNP parts.

## Bring-up checklist (for the first built board)

1. Without the devkit: apply 5 V to J1 → +5V ≈ 4.6–4.7 V; reverse it → nothing on +5V, no heat.
2. Fit the devkit, power from J1: relay must not click at power-up. Measure GPIO5 during boot
   (scope, or a meter across repeated EN resets) — must stay < 0.4 V.
3. With USB also connected (only after checking the devkit's VBUS diode): no back-feed, normal boot.
4. Trigger the door from HomeKit/web UI → single relay click, COM–NO closes for the pulse length.
5. Open/close the reed → door state follows; DOOR LED mirrors it.
6. Web UI shows plausible temperature/humidity from the on-board DHT22.
