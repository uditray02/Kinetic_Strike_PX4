import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from math import radians, cos, sin, sqrt, atan2


def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great-circle distance between two points
    using the Haversine formula.
    """
    R = 6371000  # Earth radius in meters
    d_lat = radians(lat2 - lat1)
    d_lon = radians(lon2 - lon1)
    a = sin(d_lat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(d_lon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

async def print_current_speed(drone):
    """
    Continuously print the current ground speed.
    """
    async for velocity_ned in drone.telemetry.velocity_ned():
        speed = sqrt(velocity_ned.north_m_s**2 + velocity_ned.east_m_s**2)
        print(f"Current speed: {speed:.2f} m/s")


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    status_text_task = asyncio.ensure_future(print_status_text(drone))
    speed_task = asyncio.ensure_future(print_current_speed(drone))


    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()


    await asyncio.sleep(10)  # Stabilize after takeoff

    # Target location
    target_lat = 47.3980
    target_lon = 8.5440
    target_altitude = 1  # in meters



    print("-- Printing distance to target and altitude in real-time")
    distance_task = asyncio.ensure_future(print_distance_to_target(drone, target_lat, target_lon))

    # Mission items
    mission_items = [
        MissionItem(target_lat, target_lon, target_altitude, 15, True, float('nan'), float('nan'),
                    MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'),
                    float('nan'), MissionItem.VehicleAction.NONE)
    ]

    mission_plan = MissionPlan(mission_items)

    #await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("-- Starting mission")
    await drone.mission.start_mission()

    await drone.param.set_param_float("MPC_XY_CRUISE", 15.0)  # Desired cruising speed
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 15.0)  # Maximum XY velocity
    #await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 30.0)
    #await drone.param.set_param_float("MPC_VEL_MANUAL", 15.0)
    #await drone.param.set_param_float("MPC_ACC_HOR_MAX", 15.0)
    #await drone.action.set_current_speed(15.0)  


    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))
    running_tasks = [print_mission_progress_task, distance_task, speed_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    await termination_task

    #print("-- Landing")
    #await drone.action.land()

    status_text_task.cancel()


async def print_distance_to_target(drone, target_lat, target_lon):
    """
    Continuously print the distance to the target and current altitude.
    """
    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        current_alt = position.relative_altitude_m  # Altitude relative to takeoff point

        # Calculate distance to target
        distance = haversine(current_lat, current_lon, target_lat, target_lon)

        print(f"Distance to target: {distance:.2f} meters | Altitude: {current_alt:.2f} meters")



async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    was_in_air = False
    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()
            return


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    asyncio.run(run())
