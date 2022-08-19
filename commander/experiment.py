from enum import Enum, auto, unique

@unique
class ExperimentState(Enum):
    STARTING = auto()
    RESETTING = auto()
    RUNNING = auto()
    ENDING = auto()
    ENDED = auto()