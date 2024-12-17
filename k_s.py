#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    status_text_task = asyncio.ensure_future(print_status_text(drone))

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

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Take off to 10 meters
    print("-- Taking off")
    await drone.action.takeoff()
    
    # Wait for a few seconds to allow the drone to stabilize after takeoff
    await asyncio.sleep(10)

    # Mission items
    mission_items = [
        #MissionItem(47.3977523, 8.5456111, 10, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE),  # Takeoff to 10 meters
        MissionItem(47.3980, 8.5440, 2, 20, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE)  # Strike target at 2 meters
    ]
    
    mission_plan = MissionPlan(mission_items)
    
    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("-- Starting mission")
    await drone.mission.start_mission()

    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))
    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    await termination_task

    # Landing after mission completion
    print("-- Landing")
    await drone.action.land()

    status_text_task.cancel()


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
    # Run the asyncio loop
    asyncio.run(run())
