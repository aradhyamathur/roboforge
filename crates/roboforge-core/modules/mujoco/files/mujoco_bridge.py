"""Step MuJoCo inside a ROS 2 node: publishes /joint_states and /clock,
applies /mujoco/ctrl (std_msgs/Float64MultiArray) to the actuators."""
import os

import mujoco
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MujocoBridge(Node):
    def __init__(self):
        super().__init__('mujoco_bridge')
        default = os.path.join(get_package_share_directory('@PKG@'), 'models', 'scene.xml')
        path = self.declare_parameter('model', default).value
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)

        # Hinge and slide joints only: one qpos/qvel entry each.
        self.joints = [
            j for j in range(self.model.njnt)
            if self.model.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
        ]
        self.names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in self.joints]

        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.create_subscription(Float64MultiArray, 'mujoco/ctrl', self.on_ctrl, 10)
        self.create_timer(self.model.opt.timestep, self.step)
        self.get_logger().info(f'Loaded {path}: {len(self.names)} joints, {self.model.nu} actuators')

    def on_ctrl(self, msg):
        n = min(len(msg.data), self.model.nu)
        self.data.ctrl[:n] = msg.data[:n]

    def step(self):
        mujoco.mj_step(self.model, self.data)
        stamp = Time(nanoseconds=int(self.data.time * 1e9)).to_msg()
        self.clock_pub.publish(Clock(clock=stamp))
        js = JointState()
        js.header.stamp = stamp
        js.name = self.names
        js.position = [float(self.data.qpos[self.model.jnt_qposadr[j]]) for j in self.joints]
        js.velocity = [float(self.data.qvel[self.model.jnt_dofadr[j]]) for j in self.joints]
        self.js_pub.publish(js)


def main():
    rclpy.init()
    node = MujocoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
