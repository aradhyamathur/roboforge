"""Sequence skills across the stacks this project includes.

Each skill is offered only if its module is part of the package, so the same
file works for any mix of MAVSDK, Nav2 and MoveIt. Edit build_plan() to
describe your own mission. The matching stacks must already be running
(simulator, Nav2, MoveIt, the MAVSDK bridge).
"""
import importlib.util

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

PACKAGE = __package__ or '@PKG@'


def has(module):
    return importlib.util.find_spec(f'{PACKAGE}.{module}') is not None


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        self.navigator = None
        self.arm = None

    # -- skills -----------------------------------------------------------
    def drone(self, service):
        client = self.create_client(Trigger, f'drone/{service}')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'drone/{service} not available; is the MAVSDK bridge running?')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(f'drone/{service} failed: {result.message if result else "timeout"}')
            return False
        return True

    def navigate(self, x, y, yaw=0.0):
        from .nav_goal import navigate
        from nav2_simple_commander.robot_navigator import BasicNavigator
        if self.navigator is None:
            self.navigator = BasicNavigator()
            self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        return navigate(self.navigator, x, y, yaw)

    def move_arm(self, joints):
        from .moveit_goal import ArmClient
        if self.arm is None:
            self.arm = ArmClient()
        return self.arm.move_to_joints(joints)

    # -- plan -------------------------------------------------------------
    def build_plan(self):
        plan = []
        if has('mavsdk_bridge'):
            plan.append(('drone takeoff', lambda: self.drone('takeoff')))
        if has('nav_goal'):
            plan.append(('drive to (1.5, 0.5)', lambda: self.navigate(1.5, 0.5)))
        if has('moveit_goal'):
            from .moveit_goal import PANDA_EXTENDED, PANDA_READY
            plan.append(('arm: extend', lambda: self.move_arm(PANDA_EXTENDED)))
            plan.append(('arm: ready', lambda: self.move_arm(PANDA_READY)))
        if has('mavsdk_bridge'):
            plan.append(('drone land', lambda: self.drone('land')))
        return plan

    def run(self):
        plan = self.build_plan()
        if not plan:
            self.get_logger().warn('No skill modules in this project. Add MAVSDK, Nav2 or MoveIt.')
            return False
        for index, (name, step) in enumerate(plan, start=1):
            self.get_logger().info(f'[{index}/{len(plan)}] {name}')
            if not step():
                self.get_logger().error(f'Mission stopped at: {name}')
                return False
        self.get_logger().info('Mission complete')
        return True


def main():
    rclpy.init()
    mission = Mission()
    try:
        mission.run()
    except KeyboardInterrupt:
        pass
    finally:
        for node in (mission.navigator, mission.arm, mission):
            if node is not None:
                node.destroy_node()
        rclpy.try_shutdown()
