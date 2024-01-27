import math

METERS_PER_INCH = 0.0254
INCHES_PER_METER = 39.37

# Robot Physical Characteristics with dimensional units
ROBOT_MASS = 70
DT_GEAR_RATIO = 6.0
DT_MOTORS_PER_SIDE = 2
DT_WHEEL_DIAMETER = 6
DT_WHEEL_RADIUS_INCHES = DT_WHEEL_DIAMETER / 2
DT_TRACKWIDTH_METERS = 0.546
DT_TICKS_PER_MOTOR_REV = int(2048)
DT_TICKS_PER_INCH = (DT_TICKS_PER_MOTOR_REV * DT_GEAR_RATIO) / (
    (2 * math.pi) * DT_WHEEL_DIAMETER
)
DT_TICKS_PER_METER = (DT_TICKS_PER_MOTOR_REV * DT_GEAR_RATIO) / (
    (2 * math.pi) * DT_WHEEL_DIAMETER
)
DT_KS_VOLTS = 0.22
DT_KV_VOLTSECONDS_METER = 1.98
DT_KV_VOLTSECONDS_SQUARED_METER = 0.2
DT_KP_DRIVE_VELO = 8.5
DT_KRAMSETE_B = 2
DT_KRAMSETE_ZETA = 0.7
DT_MAX_VOLTS_PATH = 10
DT_MAX_METERS_PER_SECOND = 3
DT_MAX_ACCELERATION_MPS_SQUARED = 1

ROBOT_WHEELBASE = 22
ROBOT_BUMPER_WIDTH = 3.25
ROBOT_WIDTH = 29 + (ROBOT_BUMPER_WIDTH * 2)
ROBOT_LENGTH = 26 + (ROBOT_BUMPER_WIDTH * 2)


# CAN IDS
DT_LEFT_LEADER = 1  # 1
DT_RIGHT_LEADER = 2  # 2
DT_LEFT_FOLLOWER = 3  # 3
DT_RIGHT_FOLLOWER = 4  # 4
INTAKE_MOTOR = 5  # 5
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

# Controller Mapping information
CONTROLLER_FORWARD_REAL = 1
CONTROLLER_FORWARD_SIM = 0
CONTROLLER_TURN_REAL = 4
CONTROLLER_TURN_SIM = 1
