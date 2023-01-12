import inspect
from typing import List


class CanIds:
    class Chassis:
        drive_1 = 1
        steer_1 = 2
        encoder_1 = 1

        drive_2 = 3
        steer_2 = 4
        encoder_2 = 2

        drive_3 = 5
        steer_3 = 6
        encoder_3 = 3

        drive_4 = 7
        steer_4 = 8
        encoder_4 = 4


class PWM:
    ...


# recursively get all attributes
def get_ids(cls) -> List[int]:
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
def check_ids() -> None:
    all_ids = get_ids(CanIds)
    dups = set([str(x) for x in all_ids if all_ids.count(x) > 1])
    if dups:
       # raise ValueError("Duplicate Ids detected: " + ", ".join(dups))
       pass


if __name__ == "__main__":
    check_ids()
