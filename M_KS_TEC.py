import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from math import radians, cos, sin, sqrt, atan2, asin, degrees

midpoint_lat = None
midpoint_lon = None

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    d_lat = radians(lat2 - lat1)
    d_lon = radians(lon2 - lon1)
    a = sin(d_lat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(d_lon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

# Function to calculate the new coordinates after moving a certain distance from the target
def calculate_new_position_from_target(lat1, lon1, target_lat, target_lon, distance_meters):
    earth_radius = 6371000  # Radius of the Earth in meters
    lat1_rad = radians(lat1)
    lon1_rad = radians(lon1)
    target_lat_rad = radians(target_lat)
    target_lon_rad = radians(target_lon)

    # Difference (target to current)
    delta_lat = target_lat_rad - lat1_rad
    delta_lon = target_lon_rad - lon1_rad

    # Haversine formula
    a = sin(delta_lat / 2) ** 2 + cos(lat1_rad) * cos(target_lat_rad) * sin(delta_lon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = earth_radius * c  # Distance in meters

    y = sin(delta_lon) * cos(target_lat_rad)
    x = cos(lat1_rad) * sin(target_lat_rad) - sin(lat1_rad) * cos(target_lat_rad) * cos(delta_lon)
    bearing = atan2(y, x)

    # new position by moving specifuc distance in the forward direction from the target
    new_distance = distance_meters / earth_radius  
    new_lat_rad = asin(sin(target_lat_rad) * cos(new_distance) +
                       cos(target_lat_rad) * sin(new_distance) * cos(bearing))
    new_lon_rad = target_lon_rad + atan2(sin(bearing) * sin(new_distance) * cos(target_lat_rad),
                                         cos(new_distance) - sin(target_lat_rad) * sin(new_lat_rad))

    new_lat = degrees(new_lat_rad)
    new_lon = degrees(new_lon_rad)

    return new_lat, new_lon


async def print_current_speed(drone):
    async for velocity_ned in drone.telemetry.velocity_ned():
        speed = sqrt(velocity_ned.north_m_s**2 + velocity_ned.east_m_s**2)
        print(f"Current speed: {speed:.2f} m/s")

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
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

    print("-- Waiting for drone to reach sufficient altitude")
    async for position in drone.telemetry.position():
        if position.relative_altitude_m > 1: 
            print("-- Drone reached desired altitude")
            break

    await asyncio.sleep(5)  # Additional stabilization

    # Target location
    target_lat = 47.3984274
    target_lon = 8.5422047
    target_altitude = -3  #m

    # current position
    current_lat = position.latitude_deg
    current_lon = position.longitude_deg

    # new position (100 meters forward from the target)
    new_lat, new_lon = calculate_new_position_from_target(current_lat, current_lon, target_lat, target_lon, 100)
    print(f"New coordinates (100 meters forward from target): Latitude = {new_lat}, Longitude = {new_lon}")

    # Mission
    mission_items = [
        # Ascend to 30m at the current location
        MissionItem(current_lat, current_lon, 10, 30, True, float('nan'), float('nan'),
                    MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'),
                    float('nan'), MissionItem.VehicleAction.NONE),

        # proceeding to the target location
        #MissionItem(target_lat, target_lon, 0, 30, True, float('nan'), float('nan'),
                   # MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'),
                    #float('nan'), MissionItem.VehicleAction.NONE),

        #exceeding 100m from the target(jugaad)
        MissionItem(new_lat, new_lon, target_altitude, 30, True, float('nan'), float('nan'),
                    MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'),
                    float('nan'), MissionItem.VehicleAction.NONE),
    ]

    mission_plan = MissionPlan(mission_items)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("-- Starting mission")
    await drone.mission.start_mission()
    print("-- Mission started")

    await drone.param.set_param_float("MPC_XY_CRUISE", 30.0) 
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 30.0) 

    print("-- Printing distance to target and altitude in real-time")
    distance_task = asyncio.ensure_future(print_distance_to_target(drone, target_lat, target_lon))

    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))
    running_tasks = [print_mission_progress_task, distance_task, speed_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    await termination_task


async def print_distance_to_target(drone, target_lat, target_lon):
    """
    Continuously print the distance to the target and current altitude.
    """
    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        current_alt = position.relative_altitude_m  
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

if __name__ == "__main__":
    asyncio.run(run())
