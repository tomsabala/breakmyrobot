#!/usr/bin/env bash
# Phase 0 rendering / performance gate.
#
#   scripts/gate.sh [row] [seconds]
#
# row 1 = 360 samples @ 10 Hz, max_step_size 0.002   (preferred)
# row 2 = 180 samples @ 10 Hz, max_step_size 0.002
# row 3 = 180 samples @  5 Hz, max_step_size 0.004   (floor; accepted even at RTF < 0.8)
#
# Walk the rows top to bottom, stop at the first with RTF >= 0.8, and record the
# result in docs/gate.md. The chosen row's values become the lidar and physics
# defaults for every world and robot built in later phases.
#
# RAL_NO_GPU=1 forces the llvmpipe software path (comparison measurement).
# RAL_XVFB=1 wraps gz sim in xvfb-run, the fallback if EGL yields no context.
set -euo pipefail

ROW="${1:-1}"
SECONDS_TO_RUN="${2:-60}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "$ROW" in
  1) SAMPLES=360; RATE=10; STEP=0.002 ;;
  2) SAMPLES=180; RATE=10; STEP=0.002 ;;
  3) SAMPLES=180; RATE=5;  STEP=0.004 ;;
  *) echo "unknown row: $ROW (expected 1, 2 or 3)" >&2; exit 2 ;;
esac

# The gate is a measurement, not a mission: cap it so a runaway renderer cannot
# take the desktop with it. gz sim on this scene sits well under this.
export RAL_MEM="${RAL_MEM:-4g}"

exec "$REPO_ROOT/scripts/dev.sh" bash -lc "
set -euo pipefail
SRC=/ws/ros2_ws/src/ral_description/worlds/gate_test.sdf
WORLD=/tmp/gate_row${ROW}.sdf
sed -e 's|<samples>360</samples>|<samples>${SAMPLES}</samples>|' \
    -e 's|<update_rate>10</update_rate>|<update_rate>${RATE}</update_rate>|' \
    -e 's|<max_step_size>0.002</max_step_size>|<max_step_size>${STEP}</max_step_size>|' \
    \"\$SRC\" > \"\$WORLD\"

echo \"=== row ${ROW}: ${SAMPLES} samples @ ${RATE} Hz, max_step_size ${STEP}\"
echo \"=== RAL_NO_GPU=\${RAL_NO_GPU:-0} RAL_XVFB=\${RAL_XVFB:-0}\"
ls -l /dev/dri 2>&1 || echo 'no /dev/dri in container'

RUNNER=''
if [ \"\${RAL_XVFB:-0}\" = '1' ]; then RUNNER='xvfb-run -a'; fi

\$RUNNER gz sim -s -r --headless-rendering -v2 \"\$WORLD\" > /tmp/gz_row${ROW}.log 2>&1 &
GZ_PID=\$!
sleep ${SECONDS_TO_RUN}

echo '=== gz topic -l'
timeout 15 gz topic -l || true
echo '=== one scan sample (first 400 chars of ranges)'
timeout 20 gz topic -e -t /scan_raw -n 1 > /tmp/scan_row${ROW}.txt 2>&1 || true
head -c 2000 /tmp/scan_row${ROW}.txt || true
echo
echo '=== world stats'
timeout 20 gz topic -e -t /world/gate_test/stats -n 5 || true
echo '=== gz sim log'
cat /tmp/gz_row${ROW}.log || true
kill \$GZ_PID 2>/dev/null || true
wait \$GZ_PID 2>/dev/null || true
"
