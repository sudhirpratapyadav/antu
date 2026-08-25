#!/bin/bash
# Builds antu without Gradle: a JDK plus the SDK build tools is enough.
#
#   ./build.sh          everything - pure modules, tests, then the APK
#   ./build.sh test     pure-JVM tests only (no SDK needed)
#   ./build.sh core     the Android-free modules only
#
# The module graph, and the one rule that matters:
#
#   core    pure Java. Compiled WITHOUT android.jar on the classpath, so an
#   brain   `import android.*` here is a compile error rather than a code-review
#           note. This is what keeps the planner runnable on a laptop and
#           replayable from a recording.
#
#   drivers Android-aware: hardware, and the arcos base.
#   ops     Android-aware: server, bridge, web assets.
#   app     the Android shell that wires a graph together.
set -e
cd "$(dirname "$0")"

SDK="${SDK:-${ANDROID_SDK:-$HOME/android-sdk/android-11}}"
OUT=build
MIN_API=21
ABI=arm64-v8a

PURE_MODULES="core brain"
ANDROID_MODULES="drivers ops app"

sources() {   # sources <module> [test]
  local dir="modules/$1/src/${2:-main}/java"
  [ -d "$dir" ] && find "$dir" -name '*.java' || true
}

# ---------------------------------------------------------------- pure modules
build_pure() {
  mkdir -p $OUT/classes
  local cp=""
  for m in $PURE_MODULES; do
    local src
    src=$(sources "$m")
    [ -n "$src" ] || { echo "== $m (no sources yet)"; continue; }
    echo "== javac $m (pure)"
    # No android.jar here. That omission is the enforcement mechanism.
    javac --release 8 -nowarn ${cp:+-classpath "$cp"} -d $OUT/classes $src
    cp="$OUT/classes"
  done
}

run_tests() {
  local tdir=$OUT/test-classes
  rm -rf "$tdir" && mkdir -p "$tdir"
  local src=""
  for m in $PURE_MODULES; do
    src="$src $(sources "$m") $(sources "$m" test)"
  done
  [ -n "$(echo "$src" | tr -d ' ')" ] || { echo "no tests yet"; return; }
  javac -nowarn -d "$tdir" $src
  local failed=0
  for m in $PURE_MODULES; do
    for t in $(sources "$m" test); do
      local cls
      cls=$(echo "$t" | sed "s|modules/$m/src/test/java/||; s|/|.|g; s|\.java$||")
      case "$cls" in *Test) ;; *) continue ;; esac
      echo "== $cls"
      java -cp "$tdir" "$cls" || failed=1
    done
  done
  [ $failed -eq 0 ] || { echo; echo "TESTS FAILED"; exit 1; }
}

case "${1:-all}" in
  test) build_pure >/dev/null 2>&1 || true; run_tests; exit 0 ;;
  core) rm -rf $OUT/classes; build_pure; run_tests
        mkdir -p $OUT && (cd $OUT/classes && jar cf ../antu-core.jar com)
        echo; echo "Built: $OUT/antu-core.jar"; exit 0 ;;
esac

[ -x "$SDK/aapt2" ] || { echo "SDK not found at $SDK - set SDK= or ANDROID_SDK="; exit 1; }

rm -rf $OUT/classes $OUT/dex $OUT/aar $OUT/lib
mkdir -p $OUT/classes $OUT/dex $OUT/aar $OUT/lib/$ABI

build_pure
run_tests

# ------------------------------------------------------------ vendored AARs
DEPS=""
for aar in libs/*.aar; do
  [ -e "$aar" ] || continue
  n=$(basename "$aar" .aar)
  mkdir -p "$OUT/aar/$n"
  unzip -q -o "$aar" -d "$OUT/aar/$n"
  if [ -f "$OUT/aar/$n/classes.jar" ]; then
    DEPS="$DEPS:$OUT/aar/$n/classes.jar"
    echo "== dependency $n"
  fi
  if [ -d "$OUT/aar/$n/jni/$ABI" ]; then
    cp "$OUT/aar/$n/jni/$ABI"/*.so "$OUT/lib/$ABI/" 2>/dev/null || true
  fi
done
DEPS="${DEPS#:}"

# --------------------------------------------------------- android modules
ANDROID_SRC=""
for m in $ANDROID_MODULES; do
  ANDROID_SRC="$ANDROID_SRC $(sources "$m")"
done
if [ -n "$(echo "$ANDROID_SRC" | tr -d ' ')" ]; then
  echo "== javac drivers ops app"
  # Piping javac through grep would hand the pipeline grep's exit status, and the
  # build would cheerfully package an APK from a failed compile. Capture instead.
  if ! javac --release 8 -nowarn \
        -classpath "$SDK/android.jar:$OUT/classes${DEPS:+:$DEPS}" \
        -d $OUT/classes $ANDROID_SRC > $OUT/javac.log 2>&1; then
    grep -v 'bootstrap class path' $OUT/javac.log || true
    echo
    echo "COMPILE FAILED - no APK produced"
    exit 1
  fi
  grep -v 'bootstrap class path' $OUT/javac.log | grep -v '^Note:' || true
fi

echo "== aapt2 link"
"$SDK/aapt2" link -o $OUT/app.unsigned.apk \
    --manifest modules/app/AndroidManifest.xml \
    -I "$SDK/android.jar" -A modules/app/assets --min-sdk-version $MIN_API

echo "== d8"
java -cp "$SDK/lib/d8.jar" com.android.tools.r8.D8 \
     --lib "$SDK/android.jar" --release --min-api $MIN_API --output $OUT/dex \
     $(find $OUT/classes -name '*.class') $(echo "$DEPS" | tr ':' ' ')

echo "== package"
(cd $OUT/dex && zip -q -j ../app.unsigned.apk classes.dex)
if [ -n "$(ls -A $OUT/lib/$ABI 2>/dev/null)" ]; then
  (cd $OUT && zip -q -r app.unsigned.apk lib)
fi
"$SDK/zipalign" -f 4 $OUT/app.unsigned.apk $OUT/app.aligned.apk

if [ ! -f debug.keystore ]; then
  keytool -genkeypair -keystore debug.keystore -storepass android -keypass android \
    -alias androiddebugkey -keyalg RSA -keysize 2048 -validity 10000 \
    -dname "CN=Android Debug,O=Android,C=US"
fi
java -jar "$SDK/lib/apksigner.jar" sign --ks debug.keystore \
     --ks-pass pass:android --key-pass pass:android \
     --out $OUT/antu.apk $OUT/app.aligned.apk

echo
echo "Built: $OUT/antu.apk"
