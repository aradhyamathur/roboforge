"""Arm, switch to offboard and hold 5 m. Needs PX4 SITL and the uXRCE-DDS agent running."""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class Offboard(Node):
    def __init__(self):
        super().__init__('offboard')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.ticks = 0
        self.create_timer(0.1, self.tick)

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.cmd_pub.publish(msg)

    def tick(self):
        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = self.now_us()
        self.mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.position = [0.0, 0.0, -5.0]  # NED: 5 m up
        sp.yaw = 0.0
        sp.timestamp = self.now_us()
        self.sp_pub.publish(sp)

        # PX4 wants a stream of setpoints before it accepts offboard mode.
        if self.ticks == 10:
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info('Offboard mode requested, arming')
        self.ticks += 1


def main():
    rclpy.init()
    node = Offboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
