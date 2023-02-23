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


@enum.unique
class CancoderIds(enum.IntEnum):
    swerve_1 = 1
    swerve_2 = 2
    swerve_3 = 3
    swerve_4 = 4


@enum.unique
class SparkMaxIds(enum.IntEnum):
    arm_rotation_main = 13
    arm_rotation_follower = 14
    arm_extension = 15

    intake_motor = 3


@enum.unique
class PhChannels(enum.IntEnum):
    arm_brake = 7

    intake_piston_forward = 4
    intake_piston_reverse = 5

    gripper_solenoid_forward = 0
    gripper_solenoid_reverse = 1

    spare_channel_1 = 2
    spare_channel_2 = 3


@enum.unique
class PwmChannels(enum.IntEnum):
    leds = 0


@enum.unique
class DioChannels(enum.IntEnum):
    gripper_game_piece_switch = 0

    intake_break_beam_sensor = 2

    arm_absolute_encoder = 1
