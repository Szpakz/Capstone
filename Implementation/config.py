TICK_HZ = 20    # Ticks per second for the behavior tree
DEBUG_TREE = True   # Flag to enable/disable debug output for the behavior tree
# Address for the MAVSDK drone connection in gazebo
SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"
# Time in seconds that the drone must be at a waypoint before landing
REQUIRED_TIME_SECONDS_AT_WAYPOINT = 1.0
# Number of ticks that the drone must be at a waypoint before landing
REQUIRED_TICKS_AT_WAYPOINT = int(REQUIRED_TIME_SECONDS_AT_WAYPOINT * TICK_HZ)

MAX_SPEED = 7.5  # Maximum speed of the drone in m/s
MAX_ACCELERATION = 1.5  # Maximum acceleration of the drone in m/s^2
