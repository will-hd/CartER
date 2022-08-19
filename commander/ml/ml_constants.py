"""
Contains constants for the ML setup.
"""

from enum import Enum, IntEnum, unique

@unique
class Action(IntEnum):
    LEFT = 0
    RIGHT = 1

@unique
class FailureDescriptors(str, Enum):
    MAX_STEPS_REACHED = "steps/max"

    # Try resetting environment when policy update happens, to prevent instability.
    POLICY_UPDATE_REACHED = "steps/policy_update"

    POSITION_LEFT = "position/left"
    POSITION_RIGHT = "position/right"