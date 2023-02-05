class CanIds:
    drive_1 = 1
    steer_1 = 5
    swerve_encoder_1 = 9

    drive_2 = 2
    steer_2 = 6
    swerve_encoder_2 = 10

    drive_3 = 3
    steer_3 = 7
    swerve_encoder_3 = 11

    drive_4 = 4
    steer_4 = 8
    swerve_encoder_4 = 12


class SparkMaxIds:
    arm_rotation_main = 13
    arm_rotation_follower = 14
    arm_extension = 15

    intake_motor = 16


class PcmChannels:
    arm_brake = 4

    intake_piston_forward = 6
    intake_piston_reverse = 7

    gripper_solenoid_forward = 0
    gripper_solenoid_reverse = 1


class PwmChannels:
    ...


class DioChannels:
    gripper_game_piece_switch = 0

    intake_break_beam_sensor = 2

    arm_absolute_encoder = 1


# recursively get all attributes
def get_ids(cls) -> list[int]:
    ids = []
    for name, value in vars(cls).items():
        if name.startswith("_"):
            continue
        ids.append(value)

    return ids


# enforce no duplicate ids
def check_ids(*classes) -> None:
    for cls in classes:
        ids = get_ids(cls)
        dups = {str(x) for x in ids if ids.count(x) > 1}
        if dups:
            raise ValueError("Duplicate Ids detected: " + ", ".join(dups))
