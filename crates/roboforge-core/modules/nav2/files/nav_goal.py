"""Send one navigation goal to Nav2 and wait for the result.

navigate() is importable, so other nodes (see mission.py) can reuse it.
With AMCL the robot needs an initial pose; the defaults match the
TurtleBot3 simulation. When mapping with SLAM instead, pass
localizer:=bt_navigator and set_initial_pose:=false.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def navigate(navigator, x, y, yaw=0.0):
    """Drive to (x, y, yaw) in the map frame. Returns True on success."""
    navigator.goToPose(make_pose(navigator, x, y, yaw))
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None:
            navigator.get_logger().info(
                f'distance remaining: {feedback.distance_remaining:.2f} m', throttle_duration_sec=2.0)
    return navigator.getResult() == TaskResult.SUCCEEDED


def main():
    rclpy.init()
    navigator = BasicNavigator()
    x = navigator.declare_parameter('x', 1.5).value
    y = navigator.declare_parameter('y', 0.5).value
    yaw = navigator.declare_parameter('yaw', 0.0).value
    localizer = navigator.declare_parameter('localizer', 'amcl').value
    if navigator.declare_parameter('set_initial_pose', True).value:
        start_x = navigator.declare_parameter('initial_x', -2.0).value
        start_y = navigator.declare_parameter('initial_y', -0.5).value
        navigator.setInitialPose(make_pose(navigator, start_x, start_y, 0.0))
    try:
        navigator.waitUntilNav2Active(localizer=localizer)
        ok = navigate(navigator, x, y, yaw)
        navigator.get_logger().info('goal reached' if ok else 'navigation failed')
    except KeyboardInterrupt:
        navigator.cancelTask()
    finally:
        navigator.destroy_node()
        rclpy.try_shutdown()
