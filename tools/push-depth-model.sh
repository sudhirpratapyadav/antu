#!/bin/bash
# Puts the depth model where antu can find it.
#
# The model is ~99 MB, so it lives on the device rather than inside the APK:
# bundling it would make every build push a hundred megabytes to change a line
# of Java. antu reads it from its external files directory.
#
#   ./tools/push-depth-model.sh [device-serial]
set -e
ADB="${ADB:-$HOME/platform-tools/adb}"
TARGET="/sdcard/Android/data/com.antu/files"
MODEL="da_metric_hypersim_small.onnx"
DEV="${1:-}"
[ -n "$DEV" ] && ADB="$ADB -s $DEV"

# Usually already on the phone from jarvis, which is a much faster copy than
# pushing a hundred megabytes over Wi-Fi again.
SRC="/sdcard/Android/data/com.jarvis/files/$MODEL"
$ADB shell "mkdir -p $TARGET"
if $ADB shell "[ -f $SRC ]" 2>/dev/null; then
  echo "copying from jarvis on the device..."
  $ADB shell "cp $SRC $TARGET/$MODEL"
else
  LOCAL="${LOCAL_MODEL:-./$MODEL}"
  [ -f "$LOCAL" ] || { echo "no model on the device and none at $LOCAL"; exit 1; }
  echo "pushing $LOCAL ..."
  $ADB push "$LOCAL" "$TARGET/$MODEL"
fi
$ADB shell "ls -la $TARGET/$MODEL"
