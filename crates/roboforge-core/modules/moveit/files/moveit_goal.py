"""Plan and execute a joint-space goal through MoveIt 2's move_action.

Start a MoveIt setup first, for example the Panda demo run configuration.
ArmClient is importable, so other nodes (see mission.py) can reuse it.
"""
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, PlanningOptions
from rclpy.action import ActionClient
from rclpy.node import Node

PANDA_READY = {
    'panda_joint1': 0.0, 'panda_joint2': -0.785, 'panda_joint3': 0.0, 'panda_joint4': -2.356,
    'panda_joint5': 0.0, 'panda_joint6': 1.571, 'panda_joint7': 0.785,
}
PANDA_EXTENDED = {
    'panda_joint1': 0.0, 'panda_joint2': 0.0, 'panda_joint3': 0.0, 'panda_joint4': -1.571,
    'panda_joint5': 0.0, 'panda_joint6': 1.571, 'panda_joint7': 0.785,
}


class ArmClient(Node):
    def __init__(self, group='panda_arm'):
        super().__init__('arm_client')
        self.group = group
        self.client = ActionClient(self, MoveGroup, 'move_action')

    def move_to_joints(self, joints, timeout=60.0):
        """Plan and execute. `joints` maps joint name to position in radians."""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_action server not available; is MoveIt running?')
            return False
        request = MotionPlanRequest()
        request.group_name = self.group
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        goal_constraints = Constraints()
        for name, position in joints.items():
            goal_constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(position),
                tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        request.goal_constraints = [goal_constraints]
        goal = MoveGroup.Goal(request=request, planning_options=PlanningOptions(plan_only=False))

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('goal rejected')
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        if result.result() is None:
            self.get_logger().error('timed out waiting for execution')
            return False
        code = result.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'MoveIt error code {code}')
        return code == MoveItErrorCodes.SUCCESS


def main():
    rclpy.init()
    arm = ArmClient()
    try:
        for name, pose in (('extended', PANDA_EXTENDED), ('ready', PANDA_READY)):
            ok = arm.move_to_joints(pose)
            arm.get_logger().info(f'{name}: {"reached" if ok else "failed"}')
            if not ok:
                break
    except KeyboardInterrupt:
        pass
    finally:
        arm.destroy_node()
        rclpy.try_shutdown()
