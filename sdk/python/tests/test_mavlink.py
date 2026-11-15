"""MAVLink link: vehicle and client modes against each other, and hil against a stand-in for PX4."""
import socket
import threading
import time

import numpy as np
import pytest

pytest.importorskip("mujoco")
mavutil = pytest.importorskip("pymavlink.mavutil")

import roboforge as rf  # noqa: E402


def run_in_background(app, seconds):
    th = threading.Thread(target=lambda: app.run(duration=seconds), daemon=True)
    th.start()
    return th


def wait_for(cond, timeout, step=None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if step:
            step()
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_vehicle_and_client_fly_a_mission_over_udp(free_port):
    """MuJoCo + MAVLink with no autopilot: one app is the vehicle, another is a
    MAVLink client that arms it, takes off, sends a position target and lands."""
    port = free_port()
    vehicle_link = rf.MavlinkLink(mode="vehicle", url=f"udpout:127.0.0.1:{port}")
    vehicle = rf.App(robot="quadrotor", controllers=[rf.MultirotorController(rate=250)], links=[vehicle_link], realtime=True, speed=3.0)
    client_link = rf.MavlinkLink(mode="client", url=f"udpin:127.0.0.1:{port}")
    ground = rf.App(physics={"type": "none", "rate": 50}, links=[client_link])   # no simulator on this side
    ground.setup()
    vehicle.setup()
    flight = run_in_background(vehicle, 60.0)
    try:
        assert client_link.wait_connected(10.0), "no heartbeat from the vehicle"
        pose = lambda: ground.bus.latest("state/pose")            # noqa: E731
        status = lambda: ground.bus.latest("state/status")        # noqa: E731
        assert wait_for(lambda: pose() is not None, 5.0, ground.step)
        assert pose().position[2] < 0.1 and not status().armed

        assert not client_link.takeoff(2.0), "takeoff must be refused while disarmed"
        assert client_link.arm(), "arm was not accepted"
        assert wait_for(lambda: status().armed, 5.0, ground.step)
        assert client_link.takeoff(2.0), "takeoff was not accepted"
        assert wait_for(lambda: pose().position[2] > 1.8, 20.0, ground.step), f"did not climb: {pose().position}"
        assert not client_link.disarm(), "disarming in the air must be refused"

        ground.bus.publish("cmd/setpoint", rf.Setpoint(position=np.array([1.5, -1.0, 2.5]), yaw=0.8))
        assert wait_for(lambda: np.linalg.norm(pose().position - [1.5, -1.0, 2.5]) < 0.25, 25.0, ground.step), f"did not reach target: {pose().position}"
        from roboforge.frames import yaw_of
        assert abs(yaw_of(pose().orientation) - 0.8) < 0.1, "yaw (ENU) did not survive the NED round trip"
        assert vehicle_link.flight_mode == "OFFBOARD"
        # What the client believes must match the simulator's own state.
        truth = vehicle.bus.latest("state/pose").position
        assert np.linalg.norm(truth - pose().position) < 0.1

        assert client_link.land()
        assert wait_for(lambda: not status().armed and vehicle_link.landed, 40.0, ground.step), "did not land and disarm"
        assert vehicle.bus.latest("state/pose").position[2] < 0.15
    finally:
        vehicle.stop()
        flight.join(timeout=5.0)
        vehicle.close()
        ground.close()


def test_vehicle_speaks_plain_mavlink_to_any_client(free_port):
    """A bare pymavlink connection (what a ground station is) sees the telemetry it expects."""
    port = free_port()
    app = rf.App(robot="quadrotor", controllers=[rf.MultirotorController(rate=250)],
                 links=[rf.MavlinkLink(mode="vehicle", url=f"udpout:127.0.0.1:{port}", home=(10.0, 20.0, 100.0))], realtime=True, speed=3.0)
    gcs = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    app.setup()
    flight = run_in_background(app, 30.0)
    try:
        hb = gcs.wait_heartbeat(timeout=10)
        assert hb is not None and hb.type == mavutil.mavlink.MAV_TYPE_QUADROTOR
        wanted = ("ATTITUDE", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "SYS_STATUS", "EXTENDED_SYS_STATE", "HOME_POSITION")
        seen = {}
        end = time.monotonic() + 6
        while time.monotonic() < end and not all(n in seen for n in wanted):
            m = gcs.recv_match(blocking=True, timeout=0.5)
            if m is not None:
                seen[m.get_type()] = m
        for name in wanted:
            assert name in seen, f"{name} was not sent; got {sorted(seen)}"
        g = seen["GLOBAL_POSITION_INT"]
        assert abs(g.lat / 1e7 - 10.0) < 1e-4 and abs(g.lon / 1e7 - 20.0) < 1e-4 and abs(g.alt / 1000 - 100.0) < 0.5
        assert abs(seen["ATTITUDE"].yaw - np.pi / 2) < 0.05, "facing east (ENU x) is NED yaw +90 deg"
        assert seen["EXTENDED_SYS_STATE"].landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    finally:
        app.stop()
        flight.join(timeout=5.0)
        app.close()
        gcs.close()


def test_hil_lockstep_with_a_stand_in_autopilot(free_port):
    """PX4-in-the-loop protocol: the simulator sends HIL_SENSOR and only steps when
    the autopilot answers with HIL_ACTUATOR_CONTROLS. PX4 is replaced by a thread
    that answers every sample with a fixed throttle."""
    port = free_port(socket.SOCK_STREAM)
    link = rf.MavlinkLink(mode="hil", url=f"tcpin:127.0.0.1:{port}", connect_timeout=10.0)
    app = rf.App(robot="quadrotor", links=[link], realtime=False)
    samples, stop = [], threading.Event()

    def fake_px4():
        conn = None
        while conn is None and not stop.is_set():
            try:
                conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=1)
            except Exception:
                time.sleep(0.05)
        while not stop.is_set():
            m = conn.recv_match(type="HIL_SENSOR", blocking=True, timeout=0.2)
            if m is None:
                continue
            samples.append(m)
            conn.mav.hil_actuator_controls_send(m.time_usec, [0.5] * 4 + [0.0] * 12, 129, 0)
        conn.close()

    px4 = threading.Thread(target=fake_px4, daemon=True)
    try:
        app.setup()
        px4.start()
        app.run(steps=300)
        assert len(samples) == 300, f"lockstep broken: {len(samples)} sensor samples for 300 steps"
        assert np.allclose(app.bus.latest("cmd/actuators").values, 0.5)
        first, last = samples[0], samples[-1]
        # Four rotors at half of 6.5 N on a 1 kg vehicle: 13 m/s^2 of specific force,
        # pointing up, which is -z in the autopilot's FRD body frame.
        assert abs(last.zacc + 13.0) < 0.7 and abs(last.xacc) < 0.3 and abs(last.yacc) < 0.3, (last.xacc, last.yacc, last.zacc)
        assert 940 < first.abs_pressure < 960, "pressure should match ~488 m altitude"
        assert abs(np.hypot(last.xmag, last.ymag) - 0.21) < 0.02 and abs(last.zmag - 0.43) < 0.02
        usec = [s.time_usec for s in samples]
        assert usec == sorted(usec) and usec[-1] - usec[0] == pytest.approx(299 * 2000, abs=5)
        assert app.bus.latest("state/pose").position[2] > 0.3, "throttle 0.5 is above hover, so it should climb"
    finally:
        stop.set()
        app.close()
        px4.join(timeout=3.0)


def test_hil_reports_a_missing_autopilot(free_port):
    port = free_port(socket.SOCK_STREAM)
    app = rf.App(robot="quadrotor", links=[rf.MavlinkLink(mode="hil", url=f"tcpin:127.0.0.1:{port}", connect_timeout=0.5)], realtime=False)
    app.setup()
    with pytest.raises(TimeoutError, match="no autopilot connected"):
        app.step()
    app.close()
