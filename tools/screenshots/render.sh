#!/usr/bin/env bash
# Render OLED screenshots of a StepScreen sketch on the host computer.
#
#   tools/screenshots/render.sh Truss [--scale N] [--on RRGGBB] [--off RRGGBB] [--serial]
#
# Builds sketches/<Sketch>/scenes.cpp (which #includes the example .ino)
# against a simulated board, runs it, and writes PNGs to out/<Sketch>/.
# Needs a C++17 compiler plus Adafruit_GFX_Library and AccelStepper in the
# Arduino libraries folder (override with ARDUINO_LIBS=/path/to/libraries).

set -euo pipefail

if [[ $# -lt 1 || "$1" == -* ]]; then
  sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
  exit 2
fi

SKETCH="$1"
shift

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
LIBS="${ARDUINO_LIBS:-$(cd "$REPO/.." && pwd)}"
CXX="${CXX:-c++}"

SCENES="$HERE/sketches/$SKETCH/scenes.cpp"
BUILD="$HERE/build/$SKETCH"
OUT="$HERE/out/$SKETCH"

if [[ ! -f "$SCENES" ]]; then
  echo "No scenes for '$SKETCH': create $SCENES" >&2
  exit 1
fi
for lib in Adafruit_GFX_Library AccelStepper/src; do
  if [[ ! -d "$LIBS/$lib" ]]; then
    echo "Missing $LIBS/$lib (set ARDUINO_LIBS)" >&2
    exit 1
  fi
done

# Real library sources, except drivers that touch SAMD registers or flash
# (replaced by harness/src/Fake*.cpp).
LIB_SOURCES=()
for f in "$REPO"/src/*.cpp; do
  case "$(basename "$f")" in
  StepScreenHX711.cpp | StepScreenStore.cpp) ;;
  *) LIB_SOURCES+=("$f") ;;
  esac
done

mkdir -p "$BUILD" "$OUT"
rm -f "$OUT"/*.png

# Harness headers come first so they shadow the Arduino core and hardware
# libraries; StepScreen and Adafruit_GFX headers are the real ones.
"$CXX" -std=gnu++17 -O1 -g -w \
  -DARDUINO=10819 -DSTEPSCREEN_HOST_SIM=1 \
  -I"$HERE/harness/include" \
  -I"$REPO/src" \
  -I"$LIBS/Adafruit_GFX_Library" \
  -I"$LIBS/AccelStepper/src" \
  "$HERE"/harness/src/*.cpp \
  "${LIB_SOURCES[@]}" \
  "$LIBS/Adafruit_GFX_Library/Adafruit_GFX.cpp" \
  "$LIBS/AccelStepper/src/AccelStepper.cpp" \
  "$SCENES" \
  -o "$BUILD/scenes"

"$BUILD/scenes" --out "$OUT" "$@"
