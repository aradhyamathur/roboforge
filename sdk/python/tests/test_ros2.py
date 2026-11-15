"""ROS 2 link. Needs a sourced ROS 2 environment; skipped otherwise."""
import threading
import time

import numpy as np
import pytest

pytest.importorskip("mujoco")
rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

import roboforge as rf  # noqa: E402


@pytest.fixture
def peer():
    """A second ROS node, standing in for the rest of a ROS system."""
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("peer")
    got = {}

    def listen(typ, topic):
        got[topic] = []
        node.create_subscription(typ, topic, lambda m, k=topic: got[k].append(m), 50)

    node.listen, node.got = listen, got
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    yield node
    ex.shutdown(timeout_sec=1.0)
    node.destroy_node()


def background(app, seconds):
    th = threading.Thread(target=lambda: app.run(duration=seconds), daemon=True)
    th.start()
    return th


def wait_for(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_mujoco_arm_on_ros_topics(peer):
    """MuJoCo + ROS 2: joint states, clock and tf go out; joint commands come in."""
    for typ, topic in ((JointState, "joint_states"), (Clock, "/clock"), (TFMessage, "/tf"), (Image, "camera/overview/image_raw")):
        peer.listen(typ, topic)
    cmd = peer.create_publisher(JointState, "joint_command", 10)
    app = rf.App(robot="arm", renderer=rf.MujocoRenderer(width=96, height=72, fps=5), controllers=[rf.JointPD(rate=500)],
                 links=[rf.Ros2Link()], realtime=True)
    app.setup()
    run = background(app, 30.0)
    try:
        assert wait_for(lambda: len(peer.got["joint_states"]) > 5, 10.0), "no joint_states"
        assert peer.got["joint_states"][-1].name == ["yaw", "shoulder", "elbow"]
        target = JointState(name=["yaw", "shoulder", "elbow"], position=[0.6, 0.5, -0.9])

        def reached():
            cmd.publish(target)
            return np.allclose(peer.got["joint_states"][-1].position, [0.6, 0.5, -0.9], atol=0.03)
        assert wait_for(reached, 12.0), f"arm did not follow the ROS command: {peer.got['joint_states'][-1].position}"
        seconds = lambda: [c.clock.sec + c.clock.nanosec * 1e-9 for c in peer.got["/clock"]]     # noqa: E731
        assert wait_for(lambda: seconds() and seconds()[-1] > 1.0, 5.0), "/clock should advance with simulation time"
        t = seconds()
        assert len(t) > 100 and t == sorted(t)
        assert wait_for(lambda: len(peer.got["camera/overview/image_raw"]) >= 2, 5.0), "no camera images on ROS"
        js = peer.got["joint_states"]
        stamp = js[-1].header.stamp.sec + js[-1].header.stamp.nanosec * 1e-9
        assert abs(stamp - t[-1]) < 0.1, "joint_states must be stamped in simulation time"
        assert peer.got["/tf"] and peer.got["/tf"][-1].transforms[0].child_frame_id == "base_link"
        img = peer.got["camera/overview/image_raw"]
        assert np.frombuffer(bytes(img[-1].data), np.uint8).std() > 5, "the image is blank"
        assert img and (img[-1].width, img[-1].height, img[-1].encoding) == (96, 72, "rgb8") and len(img[-1].data) == 96 * 72 * 3
    finally:
        app.stop()
        run.join(timeout=5.0)
        app.close()


def test_diffdrive_obeys_cmd_vel_and_reports_odometry(peer):
    peer.listen(Odometry, "odom")
    pub = peer.create_publisher(Twist, "cmd_vel", 10)
    app = rf.App(robot="diffdrive", controllers=[rf.DiffDriveController()], links=[rf.Ros2Link()], realtime=True)
    app.setup()
    run = background(app, 30.0)
    try:
        msg = Twist()
        msg.linear.x, msg.angular.z = 0.3, 0.4

        def moving():
            pub.publish(msg)
            od = peer.got["odom"]
            return bool(od) and abs(od[-1].twist.twist.linear.x - 0.3) < 0.05 and abs(od[-1].twist.twist.angular.z - 0.4) < 0.06
        assert wait_for(moving, 10.0), "odometry does not show the commanded velocity"
        assert wait_for(lambda: pub.publish(msg) or peer.got["odom"][-1].pose.pose.position.x > 0.2, 10.0), "the robot did not travel"
        od = peer.got["odom"][-1]
        assert abs(od.twist.twist.linear.y) < 0.05, "odometry twist must be in the robot's own frame"
        assert od.header.frame_id == "odom" and od.child_frame_id == "base_link" and od.pose.pose.position.x > 0.05
    finally:
        app.stop()
        run.join(timeout=5.0)
        app.close()


def test_ros_and_mavlink_bridge(peer, free_port):
    """ROS 2 + MAVLink: a bridge app (no simulator) turns MAVLink telemetry into ROS
    odometry and ROS pose setpoints into MAVLink position targets. The vehicle on the
    other end of the MAVLink connection is a second app (MuJoCo in vehicle mode)."""
    pytest.importorskip("pymavlink")
    port = free_port()
    vehicle = rf.App(robot="quadrotor", controllers=[rf.MultirotorController(rate=250)],
                     links=[rf.MavlinkLink(mode="vehicle", url=f"udpout:127.0.0.1:{port}")], realtime=True, speed=3.0)
    mav = rf.MavlinkLink(mode="client", url=f"udpin:127.0.0.1:{port}")
    bridge = rf.App(physics={"type": "none", "rate": 50}, links=[mav, rf.Ros2Link(node_name="bridge", enable=["odom", "setpoint"])])
    peer.listen(Odometry, "odom")
    sp_pub = peer.create_publisher(PoseStamped, "setpoint_pose", 10)
    vehicle.setup()
    bridge.setup()
    runs = [background(vehicle, 90.0), background(bridge, 90.0)]
    try:
        assert mav.wait_connected(10.0)
        assert wait_for(lambda: len(peer.got["odom"]) > 3, 10.0), "MAVLink telemetry did not reach ROS"
        assert mav.arm() and mav.takeoff(2.0)
        height = lambda: peer.got["odom"][-1].pose.pose.position.z      # noqa: E731
        assert wait_for(lambda: height() > 1.8, 20.0), f"ROS odometry does not show the climb: z={height():.2f}"
        goal = PoseStamped()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal.pose.orientation.w = 1.0, 1.5, 2.5, 1.0

        def arrived():
            sp_pub.publish(goal)
            p = peer.got["odom"][-1].pose.pose.position
            return np.linalg.norm([p.x - 1.0, p.y - 1.5, p.z - 2.5]) < 0.25
        assert wait_for(arrived, 25.0), "the ROS setpoint did not move the MAVLink vehicle"
        truth = vehicle.bus.latest("state/pose").position
        assert np.linalg.norm(truth - [1.0, 1.5, 2.5]) < 0.3, f"simulator ground truth disagrees: {truth}"
    finally:
        vehicle.stop()
        bridge.stop()
        [r.join(timeout=5.0) for r in runs]
        vehicle.close()
        bridge.close()
