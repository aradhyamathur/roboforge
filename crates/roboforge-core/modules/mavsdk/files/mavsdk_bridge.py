"""Bridge a MAVSDK vehicle into ROS 2.

Publishes drone/gps (NavSatFix) and drone/battery (Float32, 0..1) and offers
drone/takeoff and drone/land (std_srvs/Trigger), so navigation, manipulation
or mission nodes can use the vehicle without knowing MAVLink.
MAVSDK is asyncio-based, so it runs on its own event loop thread.
"""
import asyncio
import threading

import rclpy
from mavsdk import System
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


class MavsdkBridge(Node):
    def __init__(self):
        super().__init__('mavsdk_bridge')
        self.address = self.declare_parameter('system_address', 'udp://:14540').value
        self.gps_pub = self.create_publisher(NavSatFix, 'drone/gps', 10)
        self.battery_pub = self.create_publisher(Float32, 'drone/battery', 10)
        self.create_service(Trigger, 'drone/takeoff', self.on_takeoff)
        self.create_service(Trigger, 'drone/land', self.on_land)
        self.drone = System()
        self.connected = False
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_loop, daemon=True).start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._main())

    async def _main(self):
        self.get_logger().info(f'Connecting to {self.address}')
        await self.drone.connect(system_address=self.address)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break
        self.connected = True
        self.get_logger().info('Vehicle connected')
        await asyncio.gather(self._gps(), self._battery())

    async def _gps(self):
        async for pos in self.drone.telemetry.position():
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'drone'
            msg.latitude = pos.latitude_deg
            msg.longitude = pos.longitude_deg
            msg.altitude = float(pos.absolute_altitude_m)
            self.gps_pub.publish(msg)

    async def _battery(self):
        async for battery in self.drone.telemetry.battery():
            self.battery_pub.publish(Float32(data=float(battery.remaining_percent)))

    async def _takeoff(self):
        await self.drone.action.arm()
        await self.drone.action.takeoff()

    def _call(self, coroutine, response):
        """Run a MAVSDK coroutine on the asyncio thread and wait for it."""
        if not self.connected:
            coroutine.close()
            response.success = False
            response.message = 'vehicle not connected'
            return response
        try:
            asyncio.run_coroutine_threadsafe(coroutine, self.loop).result(timeout=20.0)
            response.success = True
        except Exception as err:  # MAVSDK raises ActionError with the reason
            response.success = False
            response.message = str(err) or type(err).__name__
        return response

    def on_takeoff(self, _request, response):
        return self._call(self._takeoff(), response)

    def on_land(self, _request, response):
        return self._call(self.drone.action.land(), response)


def main():
    rclpy.init()
    node = MavsdkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
