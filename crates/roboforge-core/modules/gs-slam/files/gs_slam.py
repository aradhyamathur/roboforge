"""Gaussian Splatting SLAM node skeleton.

Subscribes to a camera and odometry, selects keyframes by motion, and hands
them to SplatMap. The tracking and mapping steps are left as marked stubs:
plug in your own optimiser, or drive a checkout such as $MONOGS_DIR.
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


def image_to_numpy(msg):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f'unsupported encoding {msg.encoding}')
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // channels, channels)
    img = img[:, :msg.width, :3] if channels >= 3 else img[:, :msg.width]
    if msg.encoding.startswith('bgr'):
        img = img[..., ::-1]
    return img


class SplatMap:
    """Holds keyframes and, eventually, the Gaussian map."""

    def __init__(self):
        self.keyframes = []  # (position xyz, quaternion xyzw, HxWx3 uint8)

    def add_keyframe(self, position, orientation, image):
        self.keyframes.append((position, orientation, image))
        # TODO mapping: spawn Gaussians for unseen pixels and optimise the map
        # against the keyframe window (see ml/fit_splats.py for the gsplat call).

    def track(self, image, prior_position, prior_orientation):
        # TODO tracking: refine the pose by minimising photometric error of
        # the rendered map against `image`. Returns the odometry prior for now.
        return prior_position, prior_orientation


class GsSlam(Node):
    def __init__(self):
        super().__init__('gs_slam')
        self.kf_dist = self.declare_parameter('keyframe_distance', 0.25).value
        image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.map = SplatMap()
        self.pose = None
        self.last_kf_pos = None
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.kf_pub = self.create_publisher(Int32, 'gs_slam/keyframes', 10)
        self.get_logger().info(f'Waiting for {image_topic} and {odom_topic}')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))

    def on_image(self, msg):
        if self.pose is None:
            return
        try:
            image = image_to_numpy(msg)
        except ValueError as err:
            self.get_logger().warn(str(err), throttle_duration_sec=5.0)
            return
        position, orientation = self.map.track(image, *self.pose)
        if self.last_kf_pos is None or np.linalg.norm(position - self.last_kf_pos) >= self.kf_dist:
            self.map.add_keyframe(position, orientation, image)
            self.last_kf_pos = position
            self.kf_pub.publish(Int32(data=len(self.map.keyframes)))
            self.get_logger().info(f'keyframe {len(self.map.keyframes)}')


def main():
    rclpy.init()
    node = GsSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
