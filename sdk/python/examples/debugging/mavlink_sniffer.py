"""Listen on a MAVLink port and show what arrives: which messages, how often, from whom.

The first thing to run when a ground station, MAVSDK or your own client "does
not connect": it tells you whether anything is being sent to that port at all.

    python3 mavlink_sniffer.py                      # listens on udpin:0.0.0.0:14540 for 5 s
    python3 mavlink_sniffer.py udpin:0.0.0.0:14550 10

Only one program can listen on a UDP port. Stop the client that normally uses
the port before sniffing it.
"""
import collections
import sys
import time

from pymavlink import mavutil


def main():
    url = sys.argv[1] if len(sys.argv) > 1 else "udpin:0.0.0.0:14540"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    conn = mavutil.mavlink_connection(url, source_system=254)
    print(f"listening on {url} for {seconds:g} s ...", flush=True)
    counts, senders, last = collections.Counter(), set(), {}
    end = time.time() + seconds
    while time.time() < end:
        m = conn.recv_match(blocking=True, timeout=0.2)
        if m is None or m.get_type() == "BAD_DATA":
            continue
        counts[m.get_type()] += 1
        senders.add((m.get_srcSystem(), m.get_srcComponent()))
        last[m.get_type()] = m
    if not counts:
        print("nothing arrived. Is the vehicle running, and is it sending to this port? "
              "(A vehicle link sends to the ports in its `urls`; PX4 sends to 14540 and 14550.)")
        return 1
    print(f"\nfrom system/component: {', '.join(f'{s}/{c}' for s, c in sorted(senders))}")
    print(f"{'message':<28} {'rate':>8}")
    for name, n in counts.most_common():
        print(f"{name:<28} {n / seconds:6.1f} Hz")
    hb = last.get("HEARTBEAT")
    if hb is not None:
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"\nheartbeat: autopilot {hb.autopilot} (12 is PX4, 0 is generic), {'armed' if armed else 'disarmed'}")
    pos = last.get("LOCAL_POSITION_NED")
    if pos is not None:
        print(f"position (north, east, down): {pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
