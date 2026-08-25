#!/bin/bash
# Refreshes the vendored arcos base driver from its own repository.
#
# arcos-android is a standalone library with its own tests and its own hardware
# verification, so it is consumed as a built artifact rather than as source. Run
# this after changing it.
set -e
cd "$(dirname "$0")/.."
SRC="${ARCOS:-../arcos-android}"

[ -d "$SRC" ] || { echo "arcos-android not found at $SRC - set ARCOS="; exit 1; }
(cd "$SRC" && ./build.sh lib)
cp "$SRC/build/arcos.aar" libs/arcos.aar
cp "$SRC"/libs/usb-serial-for-android-*.aar libs/
echo "Updated libs/arcos.aar from $SRC"
