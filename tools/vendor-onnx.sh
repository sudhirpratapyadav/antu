#!/bin/bash
# Copies the ONNX Runtime AAR into libs/.
#
# Not committed: it is 27 MB, almost all of it a native library, and a binary
# that size in git history is a permanent tax on everyone who clones. The depth
# node is the only thing that needs it, and a robot without depth still drives.
#
#   ./tools/vendor-onnx.sh [path-to-onnxruntime.aar]
set -e
cd "$(dirname "$0")/.."

SRC="${1:-../redmi-note-8-pro/jarvis/libs/onnxruntime.aar}"
[ -f "$SRC" ] || {
  echo "onnxruntime.aar not found at $SRC"
  echo "Pass its path, or fetch it from Maven:"
  echo "  com.microsoft.onnxruntime:onnxruntime-android"
  exit 1
}
cp "$SRC" libs/onnxruntime.aar
echo "Vendored $(du -h libs/onnxruntime.aar | cut -f1) into libs/"
