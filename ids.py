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
    swerve_1 = 9
    swerve_2 = 10
    swerve_3 = 11
    swerve_4 = 12


@enum.unique
class SparkMaxIds(enum.IntEnum):
    arm_rotation_main = 13
    arm_rotation_follower = 14
    arm_extension = 15

    intake_motor = 16

    hall_effector1 = 5
    hall_effector2 = 6


@enum.unique
class PcmChannels(enum.IntEnum):
    arm_brake = 4

    intake_piston_forward = 6
    intake_piston_reverse = 7

    gripper_solenoid_forward = 0
    gripper_solenoid_reverse = 1


@enum.unique
class PwmChannels(enum.IntEnum):
    ...


@enum.unique
class DioChannels(enum.IntEnum):
    gripper_game_piece_switch = 0

    intake_break_beam_sensor = 2

    arm_absolute_encoder = 1

    # hall_effector = 5  # no
