#!/bin/bash
# The SDK's own MAVLink client (examples/mujoco_mavlink_quad/mission.py: arm, take off,
# fly a square in OFFBOARD, land) against a real PX4 SITL flying in MuJoCo.
#   tests/px4/client.sh <px4 build dir> <work dir> [python with the SDK]
set -u
BUILD=$(realpath "$1"); WORK=$(realpath -m "$2"); HERE=$(cd "$(dirname "$0")" && pwd)
PY=${3:-$HERE/../../sdk/python/.venv/bin/python}
mkdir -p "$WORK/rootfs"
(cd "$HERE" && exec "$PY" -m roboforge run app.yaml --headless --duration 200) > "$WORK/sim.log" 2>&1 & SIM=$!
sleep 2
(cd "$WORK/rootfs" && PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none_iris exec "$BUILD/bin/px4" -d "$BUILD/etc" -w "$WORK/rootfs" -s etc/init.d-posix/rcS) > "$WORK/px4.log" 2>&1 & PX4=$!
sleep 20          # PX4 needs a few seconds of sensor data before it will arm
(cd "$HERE/../../sdk/python/examples/mujoco_mavlink_quad" && timeout 150 "$PY" mission.py); CODE=$?
kill $PX4 2>/dev/null
wait $SIM 2>/dev/null          # the simulator stops by itself once PX4 is gone
exit $CODE
