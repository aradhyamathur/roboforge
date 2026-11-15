"""Hostile and clumsy peers. Whatever arrives over MAVLink or ROS 2, the app
keeps running, stays finite, and refuses what it should refuse."""
import math
import socket
import threading
import time

import numpy as np

import roboforge as rf
from harness import clear_error, scenario, threads


def free_port(kind=socket.SOCK_DGRAM):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def vehicle(port, **kw):
    return rf.App(robot="quadrotor", links=[rf.MavlinkLink(mode="vehicle", url=f"udpout:127.0.0.1:{port}", **kw)],
                  controllers=[rf.MultirotorController(rate=250)], realtime=True, speed=4.0)


def in_background(app, seconds):
    th = threading.Thread(target=lambda: app.run(duration=seconds), daemon=True)
    th.start()
    return th


def gcs(port):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=255)
    assert m.wait_heartbeat(timeout=10) is not None, "no heartbeat from the vehicle"
    return m, mavutil.mavlink


def pump(m, seconds):
    """Wait while reading. A client that sleeps without reading overflows its UDP
    receive buffer in about a second at these rates, and then loses the next answer."""
    end = time.time() + seconds
    while time.time() < end:
        if m.recv_match(blocking=True, timeout=0.05) is None:
            time.sleep(0.005)


def cmd(m, command, *p, wait=1.5, tries=3):
    """Send a command the way a careful client does: resend until acknowledged."""
    p = list(p) + [0] * (7 - len(p))
    for i in range(tries):
        m.mav.command_long_send(m.target_system, m.target_component, command, i, *p)
        end = time.time() + wait
        while time.time() < end:
            ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.2)
            if ack is not None and ack.command == command:
                return ack.result
    return None


def height(app):
    return float(app.bus.latest("state/pose").position[2])


def finite(app):
    p = app.bus.latest("state/pose")
    return bool(np.all(np.isfinite(p.position)) and np.all(np.isfinite(p.orientation)))


@scenario(timeout=90, needs=("pymavlink",))
def mavlink_garbage_and_floods():
    """Random bytes, truncated packets and a flood of commands on the vehicle's port."""
    port = free_port()
    app = vehicle(port)
    app.setup()
    th = in_background(app, 60)
    m, mav = gcs(port)
    rng = np.random.default_rng(1)
    for i in range(3000):                                     # garbage straight at the vehicle's socket
        m.write(rng.bytes(int(rng.integers(1, 300))))
    good = bytes(m.mav.heartbeat_encode(6, 8, 0, 0, 0).pack(m.mav))
    for i in range(500):                                      # valid start, truncated or corrupted tail
        m.write(good[:int(rng.integers(1, len(good)))] + rng.bytes(3))
    assert cmd(m, mav.MAV_CMD_COMPONENT_ARM_DISARM, 1) == 0, "the vehicle stopped answering after garbage"
    for i in range(2000):                                     # a flood of commands, most of them nonsense
        m.mav.command_long_send(m.target_system, m.target_component, int(rng.integers(0, 600)), 0, *rng.normal(0, 1e6, 7))
    pump(m, 1.0)
    while m.recv_match(blocking=False) is not None:
        pass
    # The flood contained random arm, disarm and land commands, so the state is whatever it is: start over.
    assert cmd(m, mav.MAV_CMD_COMPONENT_ARM_DISARM, 1) == 0, "arming refused after the flood"
    assert cmd(m, mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 2.0) == 0, "takeoff refused after the flood"
    t0 = time.time()
    while height(app) < 1.6 and time.time() - t0 < 15:
        pump(m, 0.1)
    assert th.is_alive(), "the app died"
    assert height(app) > 1.6 and finite(app), f"did not climb: {height(app):.2f} m"
    app.stop(); th.join(5); app.close()
    return f"survived 3500 bad packets and 2000 commands, then flew to {height(app):.2f} m"


@scenario(timeout=90, needs=("pymavlink",))
def mavlink_absurd_setpoints():
    """NaN, infinite and absurdly far targets must not corrupt the vehicle or fling it away."""
    port = free_port()
    app = vehicle(port)
    app.setup()
    th = in_background(app, 80)
    m, mav = gcs(port)
    assert cmd(m, mav.MAV_CMD_COMPONENT_ARM_DISARM, 1) == 0
    assert cmd(m, mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 2.0) == 0
    t0 = time.time()
    while height(app) < 1.7 and time.time() - t0 < 15:
        pump(m, 0.1)
    pos_mask, vel_mask = 0b0000_1001_1111_1000, 0b0000_1001_1100_0111
    def target(mask, x, y, z, vx=0, vy=0, vz=0, yaw=0.0, n=20):
        for _ in range(n):
            m.mav.set_position_target_local_ned_send(0, m.target_system, m.target_component, mav.MAV_FRAME_LOCAL_NED, mask, x, y, z, vx, vy, vz, 0, 0, 0, yaw, 0)
            time.sleep(0.02)
    nan, inf = float("nan"), float("inf")
    target(pos_mask, nan, 0, -2)
    target(pos_mask, 0, inf, -2)
    target(pos_mask, 0, 0, -2, yaw=nan)
    target(vel_mask, 0, 0, 0, vx=nan)
    target(vel_mask, 0, 0, 0, vx=1e9, vy=-1e9, n=40)         # absurd speed: obeyed only up to the speed limit
    v = np.linalg.norm(app.bus.latest("state/twist").linear)
    assert th.is_alive(), "the app died on a bad target"
    assert finite(app), "state became nan"
    assert v < 9, f"an absurd velocity target was obeyed at {v:.1f} m/s"
    pump(m, 2.0)                                              # the stream stopped: it must stop and hold, not fly on
    v = np.linalg.norm(app.bus.latest("state/twist").linear)
    assert v < 0.5, f"still moving at {v:.1f} m/s two seconds after the velocity targets stopped"
    assert 0.5 < height(app) < 6, f"height {height(app):.2f} m after the velocity targets"
    target(pos_mask, 5000.0, 0, -2, n=100)                  # 5 km away: fly towards it at a sane speed, do not flip
    v = app.bus.latest("state/twist").linear
    tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2 * (app.bus.latest("state/pose").orientation[1] ** 2 + app.bus.latest("state/pose").orientation[2] ** 2)))))
    assert th.is_alive() and finite(app)
    assert np.linalg.norm(v) < 8, f"speed {np.linalg.norm(v):.1f} m/s towards a far target"
    assert tilt < 50, f"tilted {tilt:.0f} degrees"
    assert height(app) > 1.0, f"lost height chasing a far target: {height(app):.2f} m"
    app.stop(); th.join(5); app.close()
    return f"bad targets ignored; far target flown at {np.linalg.norm(v):.1f} m/s, tilt {tilt:.0f} deg, height {height(app):.2f} m"


@scenario(timeout=90, needs=("pymavlink",))
def mavlink_command_sequences():
    """Commands in the wrong order get refused, not obeyed and not fatal."""
    port = free_port()
    app = vehicle(port)
    app.setup()
    th = in_background(app, 80)
    m, mav = gcs(port)
    ARM, TAKEOFF, LAND = mav.MAV_CMD_COMPONENT_ARM_DISARM, mav.MAV_CMD_NAV_TAKEOFF, mav.MAV_CMD_NAV_LAND
    assert cmd(m, TAKEOFF, 0, 0, 0, 0, 0, 0, 2.0) != 0, "takeoff accepted while disarmed"
    assert cmd(m, LAND) is not None, "land on the ground got no answer"
    assert cmd(m, ARM, 0) == 0, "disarm while already disarmed should be accepted"
    assert cmd(m, ARM, 1) == 0 and cmd(m, ARM, 1) == 0, "arming twice"
    assert cmd(m, TAKEOFF, 0, 0, 0, 0, 0, 0, -3.0) != 0, "takeoff to a negative altitude accepted"
    assert cmd(m, TAKEOFF, 0, 0, 0, 0, 0, 0, 2.0) == 0
    assert cmd(m, TAKEOFF, 0, 0, 0, 0, 0, 0, 3.0) == 0, "a second takeoff (new altitude) in the air"
    t0 = time.time()
    while height(app) < 2.6 and time.time() - t0 < 15:
        pump(m, 0.1)
    assert height(app) > 2.6, f"second takeoff altitude not reached: {height(app):.2f}"
    assert cmd(m, ARM, 0) != 0, "disarm accepted in the air"
    assert cmd(m, LAND) == 0 and cmd(m, LAND) == 0, "landing twice"
    assert cmd(m, TAKEOFF, 0, 0, 0, 0, 0, 0, 2.0) == 0, "takeoff during landing (go around)"
    pump(m, 2.0)
    assert height(app) > 1.0, "go-around did not climb"
    assert cmd(m, LAND) == 0
    t0 = time.time()
    while app.bus.latest("state/status").armed and time.time() - t0 < 30:
        pump(m, 0.1)
    st = app.bus.latest("state/status")
    assert not st.armed and st.landed and height(app) < 0.15, f"did not land and disarm: {st}, {height(app):.2f} m"
    assert cmd(m, ARM, 0, 21196) == 0, "forced disarm on the ground"
    assert th.is_alive() and finite(app)
    app.stop(); th.join(5); app.close()
    return "every out-of-order command answered correctly"


@scenario(timeout=90, needs=("pymavlink",))
def mavlink_clients_come_and_go():
    """Two ground stations at once; one vanishes mid-flight; a new one takes over."""
    p1, p2 = free_port(), free_port()
    app = rf.App(robot="quadrotor", links=[rf.MavlinkLink(mode="vehicle", urls=[f"udpout:127.0.0.1:{p1}", f"udpout:127.0.0.1:{p2}"])],
                 controllers=[rf.MultirotorController(rate=250)], speed=4.0)
    app.setup()
    th = in_background(app, 80)
    a, mav = gcs(p1)
    b, _ = gcs(p2)
    assert cmd(a, mav.MAV_CMD_COMPONENT_ARM_DISARM, 1) == 0
    assert cmd(b, mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 2.0) == 0, "second ground station cannot command"
    pump(b, 2.0)
    a.close()                                                # one client gone: sends to it now fail
    pump(b, 1.0)
    assert th.is_alive(), "the app died when a client went away"
    c, _ = gcs(p1)                                           # a new program on the same port
    assert cmd(c, mav.MAV_CMD_NAV_LAND) == 0, "a client that connected later cannot command"
    t0 = time.time()
    while app.bus.latest("state/status").armed and time.time() - t0 < 30:
        pump(c, 0.1)
    assert not app.bus.latest("state/status").armed, "did not land for the late client"
    app.stop(); th.join(5); app.close()
    return "two clients, one lost, one late: all served"


@scenario(timeout=60, needs=("pymavlink",))
def mavlink_client_without_a_vehicle():
    """Client mode with nothing on the other end: every call returns in bounded time, nothing hangs."""
    port = free_port()
    link = rf.MavlinkLink(mode="client", url=f"udpin:127.0.0.1:{port}")
    app = rf.App(physics={"type": "none", "rate": 50}, links=[link])
    app.setup()
    t0 = time.time()
    assert link.wait_connected(1.0) is False
    assert link.arm() is False and link.takeoff(2.0) is False and link.land() is False
    took = time.time() - t0
    assert took < 15, f"calls took {took:.0f}s with no vehicle"
    app.run(duration=1.0)
    app.close()
    out = clear_error(lambda: rf.App(physics={"type": "none"}, links=[rf.MavlinkLink(mode="client", url=f"udpin:127.0.0.1:{port}"),
                                                                    rf.MavlinkLink(mode="client", url=f"udpin:127.0.0.1:{port}")]).setup(), str(port), "already")
    return f"bounded ({took:.1f}s); port clash: {out}"


@scenario(timeout=120, needs=("pymavlink",))
def mavlink_restart_same_ports_20_times():
    """Stop and start the vehicle on the same ports repeatedly: sockets and threads are released each time."""
    port = free_port()
    th0 = threads()
    for i in range(20):
        app = vehicle(port)
        app.run(duration=0.5)
        app.close()
    time.sleep(0.5)
    assert threads() <= th0, f"{threads() - th0} threads left after 20 restarts"
    c = rf.MavlinkLink(mode="client", url=f"udpin:127.0.0.1:{port}")
    capp = rf.App(physics={"type": "none", "rate": 50}, links=[c])
    capp.setup()
    capp.close()
    return "20 restarts, no threads left, port reusable"


# --------------------------------------------------------------------------- ROS 2
def ros_peer():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    if not rclpy.ok():
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    return rclpy, rclpy.create_node("battle_peer")


def spin_pub(rclpy, node, pub, msg, seconds, hz=50):
    end = time.time() + seconds
    while time.time() < end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / hz)


@scenario(timeout=90, needs=("rclpy",))
def ros_bad_commands_do_not_kill_the_app():
    """nan in cmd_vel, joint commands of the wrong size or with unknown joint names, oversized actuator arrays."""
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    rclpy, node = ros_peer()
    base = rf.App(robot="diffdrive", links=[rf.Ros2Link()], controllers=[rf.DiffDriveController()])
    base.setup()
    th = in_background(base, 30)
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    bad = Twist(); bad.linear.x = float("nan"); bad.angular.z = float("inf")
    spin_pub(rclpy, node, pub, bad, 1.5)
    assert th.is_alive(), "nan in cmd_vel killed the app"
    p = base.bus.latest("state/pose").position
    assert np.all(np.isfinite(p)) and np.linalg.norm(p[:2]) < 0.05, f"nan cmd_vel moved the base to {p}"
    good = Twist(); good.linear.x = 0.3
    spin_pub(rclpy, node, pub, good, 3.0)
    x = base.bus.latest("state/pose").position[0]
    assert x > 0.5, f"a good cmd_vel after bad ones was not obeyed (x={x:.2f})"
    huge = Twist(); huge.linear.x = 1e12
    spin_pub(rclpy, node, pub, huge, 1.5)
    assert th.is_alive() and np.all(np.isfinite(base.bus.latest("state/pose").position)), "a huge cmd_vel broke the simulation"
    base.stop(); th.join(5); base.close()
    node.destroy_publisher(pub)

    arm = rf.App(robot="arm", links=[rf.Ros2Link()], controllers=[rf.JointPD(rate=500)])
    arm.setup()
    th = in_background(arm, 30)
    jp = node.create_publisher(JointState, "joint_command", 10)
    ap = node.create_publisher(Float64MultiArray, "actuators", 10)
    for bad in (JointState(position=[0.1] * 7), JointState(position=[float("nan")] * 3), JointState(name=["nope", "elbow"], position=[0.1, 0.2]),
                JointState(name=["yaw"], position=[0.1, 0.2, 0.3])):
        spin_pub(rclpy, node, jp, bad, 0.6)
        assert th.is_alive(), f"joint_command {bad.name} {list(bad.position)} killed the app"
    spin_pub(rclpy, node, ap, Float64MultiArray(data=[1.0] * 50), 0.6)
    spin_pub(rclpy, node, ap, Float64MultiArray(data=[float("nan")] * 3), 0.6)
    assert th.is_alive(), "a bad actuators message killed the app"
    q0 = arm.bus.latest("state/joint").position
    assert np.all(np.isfinite(q0)) and np.abs(q0).max() < 0.3, f"bad commands moved the arm: {q0}"
    spin_pub(rclpy, node, jp, JointState(name=["elbow", "yaw"], position=[-0.7, 0.4]), 3.0)       # by name, out of order
    q = arm.bus.latest("state/joint").position
    assert abs(q[0] - 0.4) < 0.05 and abs(q[2] + 0.7) < 0.05, f"joint command by name not followed: {q}"
    spin_pub(rclpy, node, jp, JointState(position=[0.2, 0.3, -0.4]), 3.0)
    q = arm.bus.latest("state/joint").position
    assert np.abs(q - [0.2, 0.3, -0.4]).max() < 0.05, f"a good joint command after bad ones was not followed: {q}"
    arm.stop(); th.join(5); arm.close()
    return "bad ROS commands dropped with a warning; good ones still obeyed; joints addressable by name"


@scenario(timeout=120, needs=("rclpy",))
def ros_restart_30_times_and_shared_context():
    """Start and stop the ROS link repeatedly in one process, next to the user's own rclpy node."""
    rclpy, node = ros_peer()
    th0 = threads()
    seen = []
    from rosgraph_msgs.msg import Clock
    node.create_subscription(Clock, "clock", lambda m: seen.append(m.clock.sec + m.clock.nanosec * 1e-9), 10)
    for i in range(30):
        app = rf.App(robot="arm", links=[rf.Ros2Link()], realtime=False)
        app.run(duration=0.2)
        app.close()
        assert rclpy.ok(), f"closing app {i} shut down the user's ROS context"
        rclpy.spin_once(node, timeout_sec=0.0)
    time.sleep(0.5)
    left = threads() - th0
    assert left <= 0, f"{left} threads left after 30 ROS apps"
    names = [n for n in node.get_node_names() if n == "roboforge"]
    assert not names, f"{len(names)} roboforge nodes still in the graph after close"
    return f"30 restarts, context intact, no nodes or threads left, {len(seen)} clock messages seen meanwhile"


@scenario(timeout=120, needs=("rclpy",))
def ros_two_apps_and_namespaces():
    """Two robots in one ROS graph: namespaced topics do not collide, and each obeys only its own commands."""
    from geometry_msgs.msg import Twist
    rclpy, node = ros_peer()
    a = rf.App(robot="diffdrive", links=[rf.Ros2Link(namespace="r1", node_name="sim1")], controllers=[rf.DiffDriveController()])
    b = rf.App(robot="diffdrive", links=[rf.Ros2Link(namespace="r2", node_name="sim2")], controllers=[rf.DiffDriveController()])
    a.setup(); b.setup()
    ta, tb = in_background(a, 30), in_background(b, 30)
    time.sleep(1.0)
    topics = dict(node.get_topic_names_and_types())
    for t in ("/r1/cmd_vel", "/r2/cmd_vel", "/r1/odom", "/r2/odom"):
        assert t in topics, f"{t} missing; have {sorted(topics)}"
    pub = node.create_publisher(Twist, "/r1/cmd_vel", 10)
    go = Twist(); go.linear.x = 0.3
    spin_pub(rclpy, node, pub, go, 3.0)
    xa, xb = a.bus.latest("state/pose").position[0], b.bus.latest("state/pose").position[0]
    assert xa > 0.5 and abs(xb) < 0.02, f"r1 moved {xa:.2f} m, r2 moved {xb:.2f} m (only r1 was commanded)"
    a.stop(); b.stop(); ta.join(5); tb.join(5); a.close(); b.close()
    return f"r1 {xa:.2f} m, r2 {xb:.2f} m"
