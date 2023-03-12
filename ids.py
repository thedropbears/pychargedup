import enum


@enum.unique
class TalonIds(enum.IntEnum):
    drive_1 = 1
    steer_1 = 5

    drive_2 = 2
    steer_2 = 6

    drive_3 = 3
    steer_3 = 7

    drive_4 = 4
    steer_4 = 8

    arm_rotation_follower = 13
    arm_rotation_main = 14


@enum.unique
class CancoderIds(enum.IntEnum):
    swerve_1 = 1
    swerve_2 = 2
    swerve_3 = 3
    swerve_4 = 4


@enum.unique
class SparkMaxIds(enum.IntEnum):
    arm_extension = 15

    intake_motor = 3


@enum.unique
class PhChannels(enum.IntEnum):
    arm_brake_rev = 7
    arm_brake_fwd = 9
    arm_extension_brake = 8

    intake_piston_forward = 4
    intake_piston_reverse = 5

    gripper_solenoid_forward = 0
    gripper_solenoid_reverse = 1

    flapper_solenoid_forward = 2
    flapper_solenoid_reverse = 3


@enum.unique
class PwmChannels(enum.IntEnum):
    leds = 0


@enum.unique
class DioChannels(enum.IntEnum):
    arm_wall_pickup_switch = 0

    intake_break_beam_sensor = 2

    arm_absolute_encoder = 1

    gripper_cube_break_beam = 3

    extension_switch_reverse = 4
