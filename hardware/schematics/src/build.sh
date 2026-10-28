#!/bin/sh
# Rebuild every schematic/diagram: TikZ/circuitikz -> PDF -> SVG + PNG
# needs: pdflatex (TeX Live with circuitikz), pdftocairo (poppler-utils)
set -e
cd "$(dirname "$0")"
for f in geiger wiring interfaces architecture control frames timing states; do
  pdflatex -interaction=nonstopmode -halt-on-error "$f.tex" > /dev/null
  pdftocairo -svg "$f.pdf" "../$f.svg"
  pdftocairo -png -r 170 -singlefile "$f.pdf" "../$f"
  cp "$f.pdf" "../$f.pdf"
done
rm -f *.aux *.log
echo "done -> hardware/schematics/*.svg|png|pdf"
