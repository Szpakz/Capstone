import asyncio
from mavsdk_grpc import System
from mavsdk_grpc import OffboardError, PositionNedYaw, VelocityNedYaw

from tree import Command, Current, build_tree, print_tree

TICK_HZ = 20
DEBUG_TREE = False
SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"


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
