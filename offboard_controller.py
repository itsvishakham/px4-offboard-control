import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw


async def run():
    """
    Autonomous flight controller for PX4 drone in Gazebo SITL.
    
    This script:
    1. Connects to PX4 flight controller via MAVLink
    2. Waits for GPS lock and system health
    3. Arms the drone
    4. Sends initial setpoints (home position)
    5. Activates Offboard mode
    6. Streams position setpoints continuously
    7. Lands safely
    8. Disarms
    
    Connection: UDP localhost:14540 (Gazebo SITL default)
    """
    
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    print("Waiting for drone health...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Health OK")
            break

    print("Arming...")
    await drone.action.arm()

    print("Sending initial setpoints...")
    for _ in range(20):
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -2.0, 0.0)
        )
        await asyncio.sleep(0.05)

    print("Starting offboard...")
    await drone.offboard.start()

    print("Flying...")
    await asyncio.sleep(5)

    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(5)

    print("Stopping offboard...")
    await drone.offboard.stop()

    print("Disarming...")
    await drone.action.disarm()


if __name__ == "__main__":
    asyncio.run(run())
