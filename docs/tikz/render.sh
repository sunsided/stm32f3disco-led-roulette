#!/usr/bin/env bash

set -euo pipefail

# pdflatex accelerometer.tex
# pdflatex magnetometer.tex
pdflatex gyroscope.tex
convert -background white -alpha remove accelerometer.pdf accelerometer.png
convert -background white -alpha remove magnetometer.pdf magnetometer.png
convert -background white -alpha remove gyroscope.pdf gyroscope.png
optipng *.png

