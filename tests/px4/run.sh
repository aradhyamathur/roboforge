#!/bin/bash
# PX4-in-the-loop end to end: MuJoCo (RoboForge SDK) as the simulator of a real PX4 SITL,
# flown by a plain MAVLink client.
#   tests/px4/run.sh <px4 build dir> <work dir> [python with the SDK]
# The PX4 build dir is one made with:  cmake -S <PX4 source> -B <dir> -G Ninja -DCONFIG=px4_sitl_default && ninja -C <dir> px4
set -u
BUILD=$(realpath "$1"); WORK=$(realpath -m "$2"); HERE=$(cd "$(dirname "$0")" && pwd)
PY=${3:-$HERE/../../sdk/python/.venv/bin/python}
mkdir -p "$WORK/rootfs" && cd "$WORK"
(cd "$HERE" && exec "$PY" -m roboforge run app.yaml --headless --duration 150) > "$WORK/sim.log" 2>&1 & SIM=$!
sleep 2
(cd "$WORK/rootfs" && PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none_iris exec "$BUILD/bin/px4" -d "$BUILD/etc" -w "$WORK/rootfs" -s etc/init.d-posix/rcS) > "$WORK/px4.log" 2>&1 & PX4=$!
"$PY" "$HERE/fly.py"; CODE=$?
kill -INT $PX4 2>/dev/null; sleep 1; kill $PX4 2>/dev/null
kill -INT $SIM 2>/dev/null; wait $SIM 2>/dev/null
echo "--- simulator"; tail -3 "$WORK/sim.log"
exit $CODE
