import inspect


class CanIds:
    class Chassis:
        drive_1 = 1
        steer_1 = 5
        encoder_1 = 9

        drive_2 = 2
        steer_2 = 6
        encoder_2 = 10

        drive_3 = 3
        steer_3 = 7
        encoder_3 = 11

        drive_4 = 4
        steer_4 = 8
        encoder_4 = 12

    class Arm:
        rotation_left = 13
        rotation_right = 14
        extension = 15


class PcmChannels:
    arm_brake = 4


class PwmChannels:
    ...


# recursively get all attributes
def get_ids(cls) -> list[int]:
    # ignore dunders
    attributes = [x for x in dir(cls) if not x.startswith("__")]
    ids = []
    for att in attributes:
        if inspect.isclass(getattr(cls, att)):
            ids.extend(get_ids(getattr(cls, att)))
        else:
            ids.append(getattr(cls, att))

    return ids


# enforce no duplicate ids
def check_ids(*classes) -> None:
    for cls in classes:
        ids = get_ids(cls)
        dups = {str(x) for x in ids if ids.count(x) > 1}
        if dups:
            raise ValueError("Duplicate Ids detected: " + ", ".join(dups))
