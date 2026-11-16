#!/usr/bin/env python3
"""Fly a real PX4 SITL with plain pymavlink: wait until it can arm, take off, hold, land.
Prints one line per stage; exit code 0 only if it climbed and landed."""
import sys
import time

from pymavlink import mavutil

ALT = 2.5
m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255)
m.wait_heartbeat(timeout=60)
print(f"heartbeat from system {m.target_system}, autopilot {m.messages['HEARTBEAT'].autopilot} (12 = PX4)", flush=True)
MAV = mavutil.mavlink
last_hb = 0.0


def pump(seconds=0.2):
    """Read for a while, keeping our ground-station heartbeat going."""
    global last_hb
    end = time.time() + seconds
    while time.time() < end:
        if time.time() - last_hb > 1.0:
            m.mav.heartbeat_send(MAV.MAV_TYPE_GCS, MAV.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = time.time()
        msg = m.recv_match(blocking=True, timeout=0.1)
        if msg is not None and msg.get_type() == "STATUSTEXT":
            print("   px4:", msg.text, flush=True)


def param(name, value, kind=MAV.MAV_PARAM_TYPE_INT32):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), value, kind)
    pump(0.3)


def command(cmd, *p, tries=1):
    p = list(p) + [0] * (7 - len(p))
    for _ in range(tries):
        m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *p)
        end = time.time() + 2
        while time.time() < end:
            pump(0.1)
            ack = m.messages.get("COMMAND_ACK")
            if ack is not None and ack.command == cmd and ack._timestamp > end - 2:
                if ack.result == 0:
                    return True
                break
        pump(1.0)
    return False


def height():
    p = m.messages.get("LOCAL_POSITION_NED")
    return None if p is None else -p.z


def armed():
    return bool(m.messages["HEARTBEAT"].base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)


def px4_mode(main, sub):
    return command(MAV.MAV_CMD_DO_SET_MODE, MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)


def encode_float_as_int(v):   # PARAM_SET carries int32 params bit-cast into the float field
    import struct
    return struct.unpack("f", struct.pack("i", v))[0]


param("COM_RC_IN_MODE", encode_float_as_int(4))          # no RC transmitter in this test
param("NAV_DLL_ACT", encode_float_as_int(0))
param("MIS_TAKEOFF_ALT", ALT, MAV.MAV_PARAM_TYPE_REAL32)

t0 = time.time()
while height() is None and time.time() - t0 < 60:
    pump(0.5)
print(f"local position estimate after {time.time() - t0:.0f}s: height {height()}", flush=True)
if height() is None:
    sys.exit("PX4 never produced a position estimate (EKF did not converge on the simulated sensors)")

ok = False
for _ in range(30):                                      # arming is refused until preflight checks pass
    px4_mode(4, 2)                                       # AUTO / TAKEOFF
    if command(MAV.MAV_CMD_COMPONENT_ARM_DISARM, 1):
        ok = True
        break
    pump(2.0)
print("armed in takeoff mode:", ok, flush=True)
if not ok:
    sys.exit("PX4 refused to arm")

t0, top = time.time(), 0.0
while time.time() - t0 < 40:
    pump(0.5)
    top = max(top, height())
    if height() > ALT - 0.4:
        break
print(f"climbed to {height():.2f} m in {time.time() - t0:.0f}s", flush=True)
climbed = height() > ALT - 0.4

t0, hs = time.time(), []
while time.time() - t0 < 8:                              # hold
    pump(0.5)
    hs.append(height())
p = m.messages["LOCAL_POSITION_NED"]
print(f"hover for 8 s: height {min(hs):.2f}..{max(hs):.2f} m, horizontal drift {abs(p.x):.2f}, {abs(p.y):.2f} m", flush=True)

px4_mode(4, 6)                                           # AUTO / LAND
t0 = time.time()
while armed() and time.time() - t0 < 60:
    pump(0.5)
print(f"landed and disarmed: {not armed()} (height {height():.2f} m, {time.time() - t0:.0f}s)", flush=True)
sys.exit(0 if climbed and not armed() else 1)
