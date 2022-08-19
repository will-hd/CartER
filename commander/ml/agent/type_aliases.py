
"""
Type alises for agents.
"""

from typing import TypedDict, Union


class CommonGoalParams(TypedDict, total=False):
    failure_position: tuple[float, float]  # m
    failure_position_velo: tuple[float, float]  # m/s

    # Potential
    track_length: Union[int, float]


GoalParams = CommonGoalParams