"""Take off to 5 m in offboard mode with MAVSDK. Needs PX4 SITL running."""
import asyncio

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


async def main():
    drone = System()
    await drone.connect(system_address='udp://:14540')
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    await drone.action.arm()
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as err:
        print(f'Offboard start failed: {err}')
        await drone.action.disarm()
        return

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(15)
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == '__main__':
    asyncio.run(main())
