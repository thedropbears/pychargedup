# NB: autonomous is tested separately in autonomous_test.py
from pyfrc.tests import (  # type: ignore
    # Running all the tests seems to segfault, but a subset seems okay?
    # test_disabled,
    # test_operator_control,
    test_practice,
)

# Make pyflakes happy about our imports.
__all__ = (
    # "test_disabled",
    # "test_operator_control",
    "test_practice",
)
