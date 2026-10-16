#!/bin/sh
# Renders every printable part to stl/ and a PNG preview of each (plus the assembly) to renders/.
# Usage: sh make_stl.sh [part ...]     (default: all parts)
set -eu
cd "$(dirname "$0")"
OSCAD=${OSCAD:-$(command -v openscad || echo /Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD)}
mkdir -p stl renders
PARTS=${*:-"base lid btn_boot btn_en lightpipe fittest template"}
for p in $PARTS; do
  "$OSCAD" -q -o "stl/$p.stl" -D "part=\"$p\"" enclosure.scad
  "$OSCAD" -q -o "renders/$p.png" --imgsize=1200,900 --viewall --autocenter --colorscheme=Tomorrow -D "part=\"$p\"" enclosure.scad
done
# Reference models (not for printing): every piece incl. the PCB, assembled and exploded. Lazy union keeps the
# pieces as separate bodies instead of fusing them.
for p in assembled exploded; do
  "$OSCAD" --enable=lazy-union --backend=manifold -q -o "stl/assembly_${p#assembled}.stl" -D "part=\"$p\"" enclosure.scad
done
mv stl/assembly_.stl stl/assembly.stl
"$OSCAD" -q -o renders/assembly.png --imgsize=1600,1200 --viewall --autocenter --colorscheme=Tomorrow -D 'part="assembly"' enclosure.scad
"$OSCAD" -q -o renders/assembly_exploded.png --imgsize=1600,1200 --viewall --autocenter --colorscheme=Tomorrow --camera=0,0,0,60,0,35,0 -D 'part="exploded"' enclosure.scad
ls stl renders
