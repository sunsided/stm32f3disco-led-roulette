#!/usr/bin/env bash

set -euo pipefail

pdflatex orientation.tex
convert -background white -alpha remove orientation.pdf orientation.png
optipng orientation.png

