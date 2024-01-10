from pyfrc.physics.units import units
import math

# Robot Physical Characteristics with dimensional units
ROBOT_MASS = 110 * units.lbs
DT_GEAR_RATIO = 6.0
DT_MOTORS_PER_SIDE = 2
DT_WHEEL_DIAMETER = 6 * units.inch
DT_WHEEL_RADIUS_INCHES = DT_WHEEL_DIAMETER.m / 2
DT_TRACKWIDTH_METERS = 0.546
DT_TICKS_PER_MOTOR_REV = int(2048)
DT_TICKS_PER_INCH = (DT_TICKS_PER_MOTOR_REV * DT_GEAR_RATIO) / (
    (2 * math.pi) * DT_WHEEL_DIAMETER.m_as(units.inch)
)
DT_TICKS_PER_METER = (DT_TICKS_PER_MOTOR_REV * DT_GEAR_RATIO) / (
    (2 * math.pi) * DT_WHEEL_DIAMETER.m_as(units.meter)
)
ROBOT_WHEELBASE = 22 * units.inch
ROBOT_BUMPER_WIDTH = 3.25 * units.inch
ROBOT_WIDTH = (23 * units.inch) + (ROBOT_BUMPER_WIDTH * 2)
ROBOT_LENGTH = (32 * units.inch) + (ROBOT_BUMPER_WIDTH * 2)


# CAN IDS
DT_LEFT_LEADER = 1  # 1
DT_RIGHT_LEADER = 2  # 2
DT_LEFT_FOLLOWER = 3  # 3
DT_RIGHT_FOLLOWER = 4  # 4
# 5
# 6
# 7
# 8
# 9
# 10
# 11
# 12
# 13
# 14

# Default Robot loop period
ROBOT_PERIOD_MS = 0.020  # 50Hz, or 20 times a second
