from enum import IntEnum, unique
from typing import TypedDict, Union

# IntEnums that map a label to the appropriate
# index of the `np.array` of the `observe` method.
@unique
class SimulatedInternalStateIdx(IntEnum):
    X = 0
    DX = 1
    THETA = 2
    DTHETA = 3

@unique
class ExperimentalPositionalInternalStateIdx(IntEnum):
    X = 0
    THETA = 1

@unique
class ExperimentalTotalLinearInternalStateIdx(IntEnum):
    X = 0
    DX = 1

# Typealiases for state index IntEnums
InternalStateIdx = Union[SimulatedInternalStateIdx, ExperimentalPositionalInternalStateIdx, ExperimentalTotalLinearInternalStateIdx]
ExternalTotalKnowledgeStateIdx = SimulatedInternalStateIdx


@unique
class ExternalPositionalKnowlegeStateIdx(IntEnum):
    X = 0
    THETA = 1

@unique
class ExternalTotalLinearKnowlegeStateIdx(IntEnum):
    X = 0
    DX = 1


ExternalStateIdx = Union[ExternalTotalKnowledgeStateIdx, ExternalPositionalKnowlegeStateIdx]


class ExternalTotalKnowledgeStateMap(TypedDict):
    x: float
    dx: float
    theta: float
    dtheta: float


class ExternalPositionalKnowledgeStateMap(TypedDict):
    x: float
    theta: float


ExternalStateMap = Union[ExternalTotalKnowledgeStateMap, ExternalPositionalKnowledgeStateMap]
