"""
### Constants.py ###

Defines constants used within the networking protocol.
"""

from enum import Enum, IntEnum, unique
from commander.network.utils import Format

DEFAULT_PORT: str = "/dev/ttyS5"
DEFAULT_BAUDRATE: int = 115200


@unique
class SetOperation(str, Enum):
    SUBTRACT = "-"
    EQUAL = "="
    ADD = "+"
    NUL = "0"

@unique
class CartID(IntEnum):
    """
    Must pack to an uint8_t.
    """
    NUL = 0
    ONE = 1


@unique
class ExperimentInfoSpecifier(IntEnum):
    """
    Enum for the type of information contained in ExperimentInfoPacket.
    Must pack to an uint8_t.
    """
    NUL = 0
    POSITION_DRIFT = 1
    FAILURE_MODE = 2
    TRACK_LENGTH_STEPS = 3
    AVAILABLE_MEMORY = 4


SPECIFIER_TO_FORMAT: dict[ExperimentInfoSpecifier, Format] = {
    ExperimentInfoSpecifier.NUL: Format.NUL,
    ExperimentInfoSpecifier.POSITION_DRIFT: Format.INT_32,
    ExperimentInfoSpecifier.FAILURE_MODE: Format.INT_8,
    ExperimentInfoSpecifier.TRACK_LENGTH_STEPS: Format.INT_32,
    ExperimentInfoSpecifier.AVAILABLE_MEMORY: Format.UINT_32,
}


class FailureMode(IntEnum):
    """
    Enum for description of the type of experiment failure.
    Must pack to an int8_t.
    """

    NUL = 0

    MAX_STEPS_REACHED = -128

    POSITION_LEFT = -1
    POSITION_RIGHT = 1

    OTHER = 127

    def describe(self) -> str:
        return MODE_TO_DESCRIPTOR[self]


MODE_TO_DESCRIPTOR: dict[FailureMode, str] = {
    FailureMode.NUL: "nul",
    FailureMode.MAX_STEPS_REACHED: "steps/max",
    FailureMode.POSITION_LEFT: "position/left",
    FailureMode.POSITION_RIGHT: "position/right",
}
