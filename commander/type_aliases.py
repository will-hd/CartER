"""
Contains related type aliases.
"""
from __future__ import annotations

from typing import Any
import numpy as np

import numpy.typing as npt
from commander.ml.ml_constants import FailureDescriptors

ExternalState = npt.NDArray[np.float64]

StateChecks = dict[FailureDescriptors, bool]

StepInfo = dict[str, Any]
StepReturn = tuple[
    ExternalState,  # Observations
    float,  # Rewards
    bool,  # Dones
    StepInfo,  # Infos,
]
