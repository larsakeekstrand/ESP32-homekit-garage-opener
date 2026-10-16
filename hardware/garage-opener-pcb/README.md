# Garage opener carrier PCB (rev 1)

KiCad 10 project for a carrier board that an **ESP32 DevKitC V4 (ESP32-WROOM-32U, 38-pin)** plugs
into. It adds a 5 V screw-terminal input (PTC fuse + Schottky), an on-board SRD-05VDC relay for the
opener's push-button input, a protected reed-switch input, an on-board DHT22, RUN/DOOR LEDs and two
board-ID straps. The GPIO map is identical to the prototype (`main/board.h`), so the same firmware
and OTA image run on both.

- Schematic: `garage-opener-pcb.kicad_sch` (PDF: `../../docs/Schematic_GarageOpener.pdf`)
- BOM: `bom.csv` (DNP parts excluded). U1 is the devkit itself; its footprint needs two
  1x19 2.54 mm female headers, which are not a separate BOM line.
- Design + bring-up checklist: `../../docs/superpowers/specs/2026-10-16-carrier-pcb-schematic-design.md`

## Checking the schematic after edits

    python3 check_schematic.py

Exports the netlist with `kicad-cli` and checks every net, footprint, the DNP ID straps, and that
each GPIO in `main/board.h` lands on the right circuit. Run ERC too:

    kicad-cli sch erc --severity-all garage-opener-pcb.kicad_sch

Expected: 0 errors; 1 warning that the embedded `MMBT2222A` differs from the library copy (it is a
flattened derived symbol) is harmless.

## Before ordering a PCB

- Measure the devkit: rows must be 25.4 mm apart (footprint `garage-opener:ESP32-DevKitC-V4_Socket`).
- Check whether the devkit has a diode on USB VBUS before using USB and J1 at the same time.
- Board ID rev 1 = `00` (R9, R11 fitted). Future revisions: fit R8 and/or R10 instead.
- Relay contacts follow KiCad's SRD symbol (J2: 1 COM, 2 NO, 3 NC). On the first board, confirm
  with a meter that COM–NO is open while the relay is unpowered.
