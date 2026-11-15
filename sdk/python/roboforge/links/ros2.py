"""ROS 2 as a link: bus signals <-> topics."""
from __future__ import annotations

import threading

import numpy as np

from .. import msgs
from ..component import Link

# Signal -> (default topic, direction in "sim" mode). In "robot" mode the
# directions flip: state comes in from ROS, commands go out to it.
TOPICS = {
    "clock": "/clock",
    "joint": "joint_states",
    "odom": "odom",
    "imu": "imu",
    "camera": "camera/{name}/image_raw",
    "cmd_vel": "cmd_vel",
    "joint_command": "joint_command",
    "actuators": "actuators",
    "setpoint": "setpoint_pose",
}
# Topic names other simulators' ROS bridges conventionally use, so a stack
# written against them does not need remapping.
PROFILES = {
    "gazebo": {"joint": "/joint_states", "odom": "/odom", "imu": "/imu", "cmd_vel": "/cmd_vel", "camera": "/{name}/image_raw"},
    "isaac": {"joint": "/joint_states", "odom": "/odom", "imu": "/imu", "cmd_vel": "/cmd_vel", "joint_command": "/joint_command", "camera": "/{name}/rgb"},
}


class Ros2Link(Link):
    """Mirror the bus on ROS 2 topics.

    `mode="sim"` (default): publish state (`/clock`, `joint_states`, `odom`,
    `/tf`, `imu`, `camera/<name>/image_raw`) and subscribe to commands
    (`cmd_vel`, `joint_command`, `actuators`, `setpoint_pose`).
    `mode="robot"`: the reverse, for driving a real or externally simulated
    robot: subscribe to its state, publish commands.

    `topics` renames any of them (`{"cmd_vel": "/robot1/cmd_vel"}`);
    `profile` ("gazebo", "isaac") applies that simulator's usual names;
    `enable` restricts which mappings are created. `link.node` (also
    `link.raw`) is the rclpy node, for anything else you want on it.
    """
    name = "ros2"

    def __init__(self, mode: str = "sim", namespace: str = "", node_name: str = "roboforge", topics: dict = None,
                 profile: str = None, enable: list = None, rate: float = 100.0, tf: bool = True,
                 frame_id: str = "odom", child_frame_id: str = "base_link"):
        if mode not in ("sim", "robot"):
            raise ValueError("Ros2Link mode must be 'sim' or 'robot'")
        if profile is not None and profile not in PROFILES:
            raise ValueError(f"unknown profile {profile!r}; choose one of {', '.join(PROFILES)}")
        self.mode, self.namespace, self.node_name, self.rate, self.tf = mode, namespace, node_name, rate, tf
        self.frame_id, self.child_frame_id = frame_id, child_frame_id
        self.topics = {**TOPICS, **PROFILES.get(profile, {}), **(topics or {})}
        self.enable = set(enable) if enable is not None else set(TOPICS)
        self.node = None
        self._next = 0.0
        self._pubs, self._cam_pubs = {}, {}
        self._owns_context = False
        # Declared up front so the wiring can be shown without starting ROS.
        state = {"clock": msgs.CLOCK, "joint": msgs.JOINT_STATE, "odom": msgs.POSE, "imu": msgs.IMU, "camera": "sensor/camera/*"}
        cmds = {"cmd_vel": msgs.CMD_VELOCITY, "joint_command": msgs.CMD_JOINT, "actuators": msgs.CMD_ACTUATORS, "setpoint": msgs.CMD_SETPOINT}
        if mode == "robot":
            state.pop("clock"), state.pop("camera")
        s_sig = [v for k, v in state.items() if k in self.enable]
        c_sig = [v for k, v in cmds.items() if k in self.enable]
        self.inputs, self.outputs = (s_sig, c_sig) if mode == "sim" else (c_sig, s_sig)

    @property
    def raw(self):
        return self.node

    def setup(self, app):
        import rclpy
        from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, Imu, JointState
        from std_msgs.msg import Float64MultiArray
        from tf2_msgs.msg import TFMessage

        self._types = dict(Clock=Clock, JointState=JointState, Odometry=Odometry, Imu=Imu, Image=Image, Twist=Twist,
                           PoseStamped=PoseStamped, Float64MultiArray=Float64MultiArray, TFMessage=TFMessage, TransformStamped=TransformStamped)
        self.bus, self.app = app.bus, app
        if not rclpy.ok():
            # Leave Ctrl+C to the app: rclpy's own handler would tear the context down mid-step.
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self._owns_context = True
        self._rclpy = rclpy
        self.node = rclpy.create_node(self.node_name, namespace=self.namespace or None)
        n, T, on = self.node, self.topics, self.enable
        sim = self.mode == "sim"

        def state(key, typ, signal):
            """State flows bus -> ROS in sim mode, ROS -> bus in robot mode."""
            if key not in on:
                return
            if sim:
                self._pubs[key] = n.create_publisher(typ, T[key], 10)
            else:
                n.create_subscription(typ, T[key], getattr(self, f"_in_{key}"), 10)

        def command(key, typ, signal):
            """Commands flow ROS -> bus in sim mode, bus -> ROS in robot mode."""
            if key not in on:
                return
            if sim:
                n.create_subscription(typ, T[key], getattr(self, f"_in_{key}"), 10)
            else:
                self._pubs[key] = n.create_publisher(typ, T[key], 10)
                self.bus.subscribe(signal, getattr(self, f"_out_{key}"))

        state("clock", Clock, msgs.CLOCK) if sim else None
        state("joint", JointState, msgs.JOINT_STATE)
        state("odom", Odometry, msgs.POSE)
        state("imu", Imu, msgs.IMU)
        command("cmd_vel", Twist, msgs.CMD_VELOCITY)
        command("joint_command", JointState, msgs.CMD_JOINT)
        command("actuators", Float64MultiArray, msgs.CMD_ACTUATORS)
        command("setpoint", PoseStamped, msgs.CMD_SETPOINT)
        if sim and self.tf and "odom" in on:
            self._pubs["tf"] = n.create_publisher(TFMessage, "/tf", 10)
        if sim and "camera" in on:
            self.bus.subscribe("sensor/camera/*", self._out_camera)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(n)
        self._spin = threading.Thread(target=self._spin_quietly, name="ros2-link", daemon=True)
        self._spin.start()

    # --- helpers -----------------------------------------------------------
    def _stamp(self, t: float):
        from builtin_interfaces.msg import Time
        return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))

    @staticmethod
    def _to_t(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    # --- bus -> ROS (state, in sim mode) -------------------------------------
    def after_step(self, t: float):
        if self.mode != "sim":
            return
        P, ty, bus = self._pubs, self._types, self.bus
        stamp = self._stamp(t)
        if "clock" in P:
            P["clock"].publish(ty["Clock"](clock=stamp))
        if t + 1e-9 < self._next:
            return
        self._next = max(self._next + 1.0 / self.rate, t)
        js = bus.latest(msgs.JOINT_STATE)
        if js is not None and "joint" in P:
            m = ty["JointState"]()
            m.header.stamp = stamp
            m.name, m.position, m.velocity, m.effort = list(js.names), js.position.tolist(), js.velocity.tolist(), js.effort.tolist()
            P["joint"].publish(m)
        pose, twist = bus.latest(msgs.POSE), bus.latest(msgs.TWIST)
        if pose is not None and "odom" in P:
            m = ty["Odometry"]()
            m.header.stamp, m.header.frame_id, m.child_frame_id = stamp, self.frame_id, self.child_frame_id
            p, q = pose.position, pose.orientation
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = float(p[0]), float(p[1]), float(p[2])
            o = m.pose.pose.orientation
            o.w, o.x, o.y, o.z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
            if twist is not None:
                from ..frames import quat_to_mat
                v = quat_to_mat(q).T @ twist.linear          # nav_msgs/Odometry twist is in the child frame
                m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = float(v[0]), float(v[1]), float(v[2])
                a = twist.angular
                m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = float(a[0]), float(a[1]), float(a[2])
            P["odom"].publish(m)
            if "tf" in P:
                tr = ty["TransformStamped"]()
                tr.header.stamp, tr.header.frame_id, tr.child_frame_id = stamp, self.frame_id, self.child_frame_id
                tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = float(p[0]), float(p[1]), float(p[2])
                r = tr.transform.rotation
                r.w, r.x, r.y, r.z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
                P["tf"].publish(ty["TFMessage"](transforms=[tr]))
        imu = bus.latest(msgs.IMU)
        if imu is not None and "imu" in P:
            m = ty["Imu"]()
            m.header.stamp, m.header.frame_id = stamp, self.child_frame_id
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (float(x) for x in imu.accel)
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (float(x) for x in imu.gyro)
            q = imu.orientation
            m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
            P["imu"].publish(m)

    def _out_camera(self, name, img):
        cam = name.rsplit("/", 1)[-1]
        pub = self._cam_pubs.get(cam)
        if pub is None:
            pub = self._cam_pubs[cam] = self.node.create_publisher(self._types["Image"], self.topics["camera"].format(name=cam), 5)
        m = self._types["Image"]()
        m.header.stamp, m.header.frame_id = self._stamp(img.t), cam
        m.height, m.width = int(img.data.shape[0]), int(img.data.shape[1])
        m.encoding, m.is_bigendian, m.step = "rgb8", 0, int(img.data.shape[1]) * 3
        m.data = np.ascontiguousarray(img.data, np.uint8).tobytes()
        pub.publish(m)

    # --- ROS -> bus ----------------------------------------------------------
    def _in_cmd_vel(self, m):
        if not self.finite([m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z]):
            return self.reject(f"a {self.topics['cmd_vel']} message", "it contains nan or inf")
        self.bus.publish(msgs.CMD_VELOCITY, msgs.velocity_command((m.linear.x, m.linear.y, m.linear.z), (m.angular.x, m.angular.y, m.angular.z)), "ros2")

    def _in_joint_command(self, m):
        topic = f"a {self.topics['joint_command']} message"
        names, pos = list(m.name), np.array(m.position, float)
        if not self.finite(pos, m.velocity, m.effort):
            return self.reject(topic, "it contains nan or inf")
        joints = list(getattr(self.app.physics, "joint_names", []) or [])
        if joints:
            if names:
                # Addressed by name: any subset, in any order. The other joints keep their target.
                if len(names) != len(pos):
                    return self.reject(topic, f"it names {len(names)} joints but gives {len(pos)} positions")
                unknown = [n for n in names if n not in joints]
                if unknown:
                    return self.reject(topic, f"unknown joint {', '.join(unknown)}; this robot has {', '.join(joints)}")
                prev = self.bus.latest(msgs.CMD_JOINT)
                state = self.bus.latest(msgs.JOINT_STATE)
                full = (np.array(prev.position, float) if prev is not None and len(prev.position) == len(joints)
                        else state.position.copy() if state is not None else np.zeros(len(joints)))
                for n, p in zip(names, pos):
                    full[joints.index(n)] = p
                pos, names = full, joints
            elif len(pos) != len(joints):
                return self.reject(topic, f"it has {len(pos)} positions but this robot has {len(joints)} joints ({', '.join(joints)}); "
                                          f"send all of them in that order, or fill in `name`")
            else:
                names = joints
        self.bus.publish(msgs.CMD_JOINT, msgs.JointCommand(names=names, position=pos), "ros2")

    def _in_actuators(self, m):
        topic = f"a {self.topics['actuators']} message"
        values = np.array(m.data, float)
        if not self.finite(values):
            return self.reject(topic, "it contains nan or inf")
        nu = len(getattr(self.app.physics, "actuator_names", values))
        if len(values) != nu:
            return self.reject(topic, f"it has {len(values)} values but this robot has {nu} actuators")
        self.bus.publish(msgs.CMD_ACTUATORS, msgs.ActuatorCommand(values=values), "ros2")

    def _in_setpoint(self, m):
        from ..frames import yaw_of
        p, o = m.pose.position, m.pose.orientation
        if not self.finite([p.x, p.y, p.z, o.w, o.x, o.y, o.z]):
            return self.reject(f"a {self.topics['setpoint']} message", "it contains nan or inf")
        if abs(o.w) + abs(o.x) + abs(o.y) + abs(o.z) < 1e-9:
            o.w = 1.0           # an unset orientation means "no yaw preference"
        self.bus.publish(msgs.CMD_SETPOINT, msgs.Setpoint(position=np.array([p.x, p.y, p.z]), yaw=yaw_of([o.w, o.x, o.y, o.z])), "ros2")

    def _in_joint(self, m):
        if not self.finite(m.position, m.velocity, m.effort):
            return self.reject(f"a {self.topics['joint']} message", "it contains nan or inf")
        self.bus.publish(msgs.JOINT_STATE, msgs.JointState(names=list(m.name), position=np.array(m.position, float), velocity=np.array(m.velocity, float),
                                                           effort=np.array(m.effort, float), t=self._to_t(m.header.stamp)), "ros2")

    def _in_odom(self, m):
        from ..frames import quat_to_mat
        p, o, t = m.pose.pose.position, m.pose.pose.orientation, self._to_t(m.header.stamp)
        q = np.array([o.w, o.x, o.y, o.z])
        l, a = m.twist.twist.linear, m.twist.twist.angular
        if not self.finite([p.x, p.y, p.z], q, [l.x, l.y, l.z, a.x, a.y, a.z]) or np.linalg.norm(q) < 1e-9:
            return self.reject(f"a {self.topics['odom']} message", "it contains nan or inf, or an all-zero orientation")
        self.bus.publish(msgs.POSE, msgs.Pose(position=np.array([p.x, p.y, p.z]), orientation=q, t=t), "ros2")
        self.bus.publish(msgs.TWIST, msgs.Twist(linear=quat_to_mat(q) @ np.array([l.x, l.y, l.z]), angular=np.array([a.x, a.y, a.z]), t=t), "ros2")

    def _in_imu(self, m):
        a, g, o = m.linear_acceleration, m.angular_velocity, m.orientation
        if not self.finite([a.x, a.y, a.z, g.x, g.y, g.z, o.w, o.x, o.y, o.z]):
            return self.reject(f"a {self.topics['imu']} message", "it contains nan or inf")
        self.bus.publish(msgs.IMU, msgs.Imu(accel=np.array([a.x, a.y, a.z]), gyro=np.array([g.x, g.y, g.z]),
                                            orientation=np.array([o.w, o.x, o.y, o.z]), t=self._to_t(m.header.stamp)), "ros2")

    # --- bus -> ROS (commands, in robot mode) --------------------------------
    def _out_cmd_vel(self, _n, c):
        m = self._types["Twist"]()
        m.linear.x, m.linear.y, m.linear.z = (float(x) for x in c.linear)
        m.angular.x, m.angular.y, m.angular.z = (float(x) for x in c.angular)
        self._pubs["cmd_vel"].publish(m)

    def _out_joint_command(self, _n, c):
        m = self._types["JointState"]()
        m.name, m.position, m.velocity, m.effort = list(c.names), np.asarray(c.position, float).tolist(), np.asarray(c.velocity, float).tolist(), np.asarray(c.effort, float).tolist()
        self._pubs["joint_command"].publish(m)

    def _out_actuators(self, _n, c):
        self._pubs["actuators"].publish(self._types["Float64MultiArray"](data=np.asarray(c.values, float).tolist()))

    def _out_setpoint(self, _n, sp):
        if sp.position is None:
            return
        m = self._types["PoseStamped"]()
        m.header.frame_id = self.frame_id
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(x) for x in sp.position)
        m.pose.orientation.w, m.pose.orientation.z = float(np.cos(sp.yaw / 2)), float(np.sin(sp.yaw / 2))
        self._pubs["setpoint"].publish(m)

    def _spin_quietly(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self._exec.spin()
        except ExternalShutdownException:
            pass                         # Ctrl+C: rclpy shut the context down before we did

    def close(self):
        if self.node is None:
            return
        self._exec.shutdown(timeout_sec=1.0)
        self._spin.join(timeout=2.0)     # the spin thread must be out of rcl before the node goes away
        self.node.destroy_node()
        self.node = None
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()

    def describe(self):
        return {**super().describe(), "mode": self.mode, "topics": {k: self.topics[k] for k in sorted(self.enable) if k in self.topics}}
