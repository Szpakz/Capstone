import asyncio
import math
from config import DEBUG_TREE, SYSTEM_ADDRESS, TICK_HZ, MAX_SPEED, MAX_ACCELERATION
from dataclasses import dataclass
from mavsdk_grpc import System
from mavsdk_grpc.offboard import PositionNedYaw, VelocityNedYaw, OffboardError

from BehaviorTree.tree import Command, Current, build_tree, print_tree


@dataclass
class SmoothMovement:
    # Smooth outgoing movement parameters for the drone
    current: Current
    max_speed: float = MAX_SPEED
    max_acceleration: float = MAX_ACCELERATION

    north: float = 0.0
    east: float = 0.0

    velocity_north: float = 0.0
    velocity_east: float = 0.0

    def __post_init__(self):
        self.north = self.current.north
        self.east = self.current.east

    def update(self, target_north: float, target_east: float, target_down: float, yaw, dt):
        desired_north = target_north - self.north
        desired_east = target_east - self.east
        dist = math.hypot(desired_north, desired_east)

        if dist > 0.01:
            speed = min(self.max_speed, math.sqrt(
                2 * self.max_acceleration * dist))
            desired_velocity_north = (desired_north / dist) * speed
            desired_velocity_east = (desired_east / dist) * speed
        else:
            desired_velocity_north = 0.0
            desired_velocity_east = 0.0

        # limit accel
        delta_v_north = desired_velocity_north - self.velocity_north
        delta_v_east = desired_velocity_east - self.velocity_east

        change = math.hypot(delta_v_north, delta_v_east)
        max_change = self.max_acceleration * dt
        if change > max_change:
            scale = max_change / change
            delta_v_north *= scale
            delta_v_east *= scale

        self.velocity_north += delta_v_north
        self.velocity_east += delta_v_east

        # update commanded position
        self.north += self.velocity_north * dt
        self.east += self.velocity_east * dt

        return PositionNedYaw(self.north, self.east, target_down, yaw)

# ------------------------------------------- Current State


async def current_position(drone: System, current: Current):
    # get current drone position
    async for pos in drone.telemetry.position_velocity_ned():
        current.north = pos.position.north_m
        current.east = pos.position.east_m
        current.down = pos.position.down_m


async def current_yaw(drone: System, current: Current):
    # get current drone yaw
    async for attitude in drone.telemetry.attitude_euler():
        current.yaw = attitude.yaw_deg


async def current_battery(drone: System, current: Current):
    # Get current battery level.Convert to fraction if given in percentage
    async for battery in drone.telemetry.battery():
        temp = battery.remaining_percent
        current.battery = temp/100.0 if temp > 1.0 else temp

# ------------------------------------------- Drone Movement


async def drone_action(drone: System, current: Current, cmd: Command, movement: SmoothMovement):
    if cmd.command_type == "takeoff":
        # drone is in offboard mode, so we can just set the position to the cruise altitude
        target = movement.update(
            current.north, current.east, -current.cruise_alt, current.yaw, 1.0 / TICK_HZ)
        await drone.offboard.set_position_ned(target)

    elif cmd.command_type == "position":
        target = movement.update(
            cmd.north, cmd.east, cmd.down, cmd.yaw, 1.0 / TICK_HZ)
        await drone.offboard.set_position_ned(target)

    elif cmd.command_type == "velocity":
        await drone.offboard.set_velocity_ned(VelocityNedYaw(cmd.velocity_north, cmd.velocity_east, cmd.velocity_down, cmd.velocity_yaw))

    elif cmd.command_type == "land":
        await land_and_wait(drone)
        return True  # indicate that we have landed

    elif cmd.command_type == "hover":
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    elif cmd.command_type == "none":
        pass

    elif cmd.command_type == "return_home":
        target = movement.update(
            0, 0, -current.cruise_alt, current.yaw, 1.0 / TICK_HZ)
        await drone.offboard.set_position_ned(target)

    else:
        print(f"Unknown command type: {cmd.command_type}, Defaulting to hover")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))


async def land_and_wait(drone: System):
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    print("Drone has landed.")


async def main():
    # Create a drone and connect to it
    drone = System()
    await drone.connect(system_address=SYSTEM_ADDRESS)

    print("Waiting for drone to connect")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone Connected")
            break

    print("Waiting for drone to have a global position estimate")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    current = Current()
    tree = build_tree(current)

    movement = SmoothMovement(current)

    tasks = [
        asyncio.create_task(current_position(drone, current)),
        asyncio.create_task(current_yaw(drone, current)),
        asyncio.create_task(current_battery(drone, current))
    ]

    await asyncio.sleep(1)  # wait for initial telemetry data

    print("Arming drone")
    await drone.action.arm()

    # Start offboard mode (PX4  needs a setpoint before starting offboard)
    await drone.offboard.set_position_ned(PositionNedYaw(current.north, current.east, current.down, current.yaw))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        # if offboard fails to start disarm and cancel tasks
        print(
            f"Starting offboard mode failed with error code: {error._result.result}")
        print("Disarming drone")
        await drone.action.disarm()
        for task in tasks:
            task.cancel()
        return

    print("Offboard mode started, running tree")

    landed = False
    tick_count = 0  # counts the number of ticks since the start of the mission
    try:
        while True:
            current.command = Command()
            tree.tick()
            if DEBUG_TREE and tick_count % TICK_HZ == 0:  # print tree every second
                print_tree(tree)
            tick_count += 1

            if await drone_action(drone, current, current.command, movement):
                landed = True
                break
            await asyncio.sleep(1.0 / TICK_HZ)
    finally:
        if not landed:
            await land_and_wait(drone)
        for task in tasks:
            task.cancel()
        await drone.offboard.stop()

if __name__ == "__main__":
    asyncio.run(main())
