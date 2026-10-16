#!/bin/sh
# Writes Aisler-ready Gerbers + drill + zip, and STEP/STL 3D models, into fab/.
set -eu
cd "$(dirname "$0")"
B=garage-opener-pcb.kicad_pcb
rm -rf fab && mkdir -p fab/gerbers
kicad-cli pcb export gerbers -o fab/gerbers/ --layers F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts --subtract-soldermask "$B"
kicad-cli pcb export drill -o fab/gerbers/ --format excellon --excellon-separate-th --generate-map --map-format gerberx2 "$B"
(cd fab/gerbers && zip -q ../garage-opener-pcb-gerbers.zip *)
kicad-cli pcb export step -o fab/garage-opener-pcb.step --subst-models --force "$B"
kicad-cli pcb export stl  -o fab/garage-opener-pcb.stl  --subst-models --force "$B"
ls fab fab/gerbers
