#!/bin/sh
# Renders every printable part to stl/ and a PNG preview of each (plus the assembly) to renders/.
# Usage: sh make_stl.sh [part ...]     (default: all parts)
set -eu
cd "$(dirname "$0")"
OSCAD=${OSCAD:-$(command -v openscad || echo /Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD)}
mkdir -p stl renders
PARTS=${*:-"bracket base lid btn_boot btn_en lightpipe fittest"}
for p in $PARTS; do
  "$OSCAD" -q -o "stl/$p.stl" -D "part=\"$p\"" enclosure.scad
  "$OSCAD" -q -o "renders/$p.png" --imgsize=1200,900 --viewall --autocenter --colorscheme=Tomorrow -D "part=\"$p\"" enclosure.scad
done
"$OSCAD" -q -o renders/assembly.png --imgsize=1600,1200 --viewall --autocenter --colorscheme=Tomorrow -D 'part="assembly"' enclosure.scad
ls stl renders
