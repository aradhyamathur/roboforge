"""Fly the simulated vehicle over MAVLink: arm, take off, fly a square, land.

Run the app first (python3 -m roboforge run app.yaml), then this script.
It is an ordinary MAVLink client, so it would fly a real PX4 the same way.
"""
import numpy as np
import roboforge as rf


def main():
    mav = rf.MavlinkLink(mode="client", url="udpin:127.0.0.1:14540")
    app = rf.App(physics={"type": "none", "rate": 50}, links=[mav])
    app.setup()

    def position():
        pose = app.bus.latest("state/pose")
        return None if pose is None else pose.position

    def fly_to(x, y, z, timeout=40.0):
        target = np.array([x, y, z], float)
        app.bus.publish("cmd/setpoint", rf.Setpoint(position=target))
        end = app.time + timeout
        app.run(until=lambda a: a.time > end or np.linalg.norm(position() - target) < 0.3)
        print(f"at {np.round(position(), 2)}", flush=True)

    print("waiting for the vehicle...", flush=True)
    if not mav.wait_connected(60.0):
        raise SystemExit("no vehicle heard on udp 14540. Is the app running?")
    app.run(until=lambda a: position() is not None)
    print("armed:", mav.arm(), flush=True)
    print("takeoff:", mav.takeoff(2.0), flush=True)
    app.run(until=lambda a: position()[2] > 1.7)
    for corner in [(2, 0, 2), (2, 2, 2), (0, 2, 2), (0, 0, 2)]:
        fly_to(*corner)
    print("landing:", mav.land(), flush=True)
    app.run(until=lambda a: not app.bus.latest("state/status").armed)
    print("landed and disarmed", flush=True)
    app.close()


if __name__ == "__main__":
    main()
