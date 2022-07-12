from typing import Any, Type

from commander.ml.agent.agent import (
    CartpoleAgent,
    ExperimentalCartpoleAgent,
    SimulatedCartpoleAgent,
)
from commander.ml.agent.goal import AgentGoalMixinBase, AgentSwingupGoalMixin, AgentTimeGoalMixin
from commander.ml.agent.state_specification import (
    AgentPositionalKnowledgeStateSpecMixin,
    AgentStateSpecBaseMixin,
    AgentTotalKnowledgeStateSpecMixin,
)


# Any to stop 'only concrete class can be given' mypy error.
# See: https://github.com/python/mypy/issues/5374
def make_agent(
    agent: Any,
    state_spec: Type[AgentStateSpecBaseMixin],
    goal: Any,
    *args: Any,
    **kwargs: Any,
) -> CartpoleAgent:
    class Agent(goal, state_spec, agent):  # type: ignore [valid-type, misc]
        ...

    return Agent(*args, **kwargs)


__all__ = (
    "make_agent",
    "CartpoleAgent",
    "SimulatedCartpoleAgent",
    "ExperimentalCartpoleAgent",
    "AgentGoalMixinBase",
    "AgentTimeGoalMixin",
    "AgentSwingupGoalMixin",
    "AgentStateSpecBaseMixin",
    "AgentTotalKnowledgeStateSpecMixin",
    "AgentPositionalKnowledgeStateSpecMixin",
)
