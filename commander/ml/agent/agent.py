from __future__ import annotations

import logging
from collections import deque
from collections.abc import Mapping
from random import random
from time import sleep, time
from typing import TYPE_CHECKING, Any, Deque, Optional, Type, TypeVar, cast, Union
from enum import IntEnum

import numpy as np

from gym import spaces

from commander.experiment import ExperimentState
from commander.type_aliases import ExternalState, StateChecks, StepInfo
from commander.network.network_constants import SetOperation, CartID
from commander.ml.ml_constants import Action, FailureDescriptors

from commander.ml.agent.type_aliases import GoalParams

from commander.network.network import NetworkManager
from commander.network.protocol import CartSpecificPacket, ObservationPacket, SetVelocityPacket, SetPositionPacket
from commander.utils import FrequencyTicker

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from commander.ml.environment import CartpoleEnv, ExperimentalCartpoleEnv


# For reference, currently unused
class SimulatedInternalStateIdx(IntEnum):
    X = 0
    DX = 1

class CartpoleAgent():
    """
    Base class for all Cartpole Agents.
    Description
    -----------
    - Responsible for taking action
        - Sending packet
    - Checking state for failure
    - Getting reward

    Attributes
    ----------
    _state: (ExternalState) Array of the state [position, velocity]
    port: (str)
    baudrate: (end)
    env: (gym.Env) Gym environment
    settled_x_threshold: (float) Positions must be within this threshold to be settled 
    INTERVAL TIMERS:
        observation maximum_interval: (float)
        action_minimum_interval: (float) 
        action_maximum_interval: (float)
        last_observation_interval: (float)
        last_observation_time: (int) 
        last_action_time: (float)
    max_steps: (int) Maximum number of steps before failure
    policy_update_steps: (int)

    goal_params: (dict)
        failure_position: (tuple) Currently unused in EXPERIMENT
        failure_position_velo: (tuple) Currently unused in EXPERIMENT
        track_length: (int) Length of rail in stesp as read by stepper motor
    
    agent_observation_buffer: (Deque[ExternalState]) Deque containing previous
        observations. 
    
    action_counter?
    
    Methods
    -------
    step
    set_environment
        Sets CartpoleEnv to agent.
    reward
        Takes in a state and returns the appropriate reward.
    check_state
        Takes in a state and returns True if the state is not failed.
    observe
        Returns the current external state of the environment as seen by the agent
    observe_as_dict
        Returns external state as dictionary
    setup
        Called once when first initialised - not after reset
    _pre_reset
        Resets various counters before reset that returns initial state
    reset
        Sets counters to zero and returns initial observation
    
    absorb_packet
    is_settled


    """

    failure_position: tuple[float, float]  # m
    failure_position_velo: tuple[float, float]  # m/s
    track_length: Union[float, int]
    
    _DEFAULT_GOAL_PARAMS: GoalParams = {
        "track_length": 1,
        "failure_position": (-np.inf, np.inf),  # m
        "failure_position_velo": (-np.inf, np.inf),  # m/s
    }

    def __init__(
        self: CartpoleAgentT,
        max_steps: int = 3000,
        settled_x_threshold: float = 5.0,
        settled_down_theta_threshold: float = 2.0,
        settled_up_theta_threshold: float = 2.0,
        observation_maximum_interval: int = 10 * 1000,  # us
        action_minimum_interval: float = 0.003,  # s
        action_maximum_interval: float = 0.010,
    ):

        self.goal_params: GoalParams = {
        "track_length": 25000, # Actually ~25000
        "failure_position": (-0.5, 0.5),  # m
        "failure_position_velo": (-np.inf, np.inf),  # m/s
        }

        self.cart_id = CartID.ONE
        self.max_steps = max_steps

        self.policy_update_steps: int = 0

        self.env: Optional[CartpoleEnv[CartpoleAgentT]] = None

        self.info: Mapping[str, Any] = {}

        # This is private for a reason
        self._state: Optional[ExternalState] = np.zeros(4) # = None
        self.observation_buffer_size = 100

        # Settled thresholds
        self.settled_x_threshold = settled_x_threshold
        self.settled_down_theta_threshold = settled_down_theta_threshold
        self.settled_up_theta_threshold = settled_up_theta_threshold


        # Timing intervals
        self.action_minimum_interval = action_minimum_interval
        self.action_maximum_interval = action_maximum_interval
        self.observation_maximum_interval = observation_maximum_interval

        # Used for estimating angular velocity
        self.dTime = 1

        self.failure_angle = [1796, 2300]

        self.setup()
        self.update_goal(self.goal_params)
        self._pre_reset()
    
    def _pre_reset(self) -> None:
        """
        Called by environment prior to the reset that needs to return
        observations.
        Resets:
            agent_obsrvation_buffer, last_observation_interval
            last_observation_time, last_action_time, action_freq_ticker
            _state, angle_offset

        """
        
        self.agent_observation_buffer: Deque[ExternalState] = deque(maxlen=self.observation_buffer_size)

        # Initialise so non empty for first obs (otherwise get TypeError: 'NoneType' object is not subscriptable)
        self.agent_observation_buffer.append([0,0,0,0])
        self.agent_observation_buffer.append([0,0,0,0])
        # print(self.agent_observation_buffer[-2][2])


        self.last_observation_interval: float = 0
        self.last_observation_time: int = 0
        self.last_action_time: float = 0.0
        self.action_freq_ticker.clear()
        self._state: Optional[ExternalState] = np.zeros(4) # = None

    def reset(self) -> ExternalState:
        """
        Called when the agent is reset.
        Returns:
            External state of agent
        """
        self.steps: int = 0

        # random_position = np.random.randint(low=2000, high=22000)
        # print(f"Random position: {random_position}")
        # pkt = SetPositionPacket(operation=SetOperation.EQUAL, cart_id=CartID.ONE, value=random_position, actobs_tracker=1)
        # self.network_manager.send_packet(pkt)

        
        # random_start = np.random.randint(low=-1000, high=1000)
        # print(f"Random starting velocity: {random_start}")
        # # Random start conditions
        # velo_pkt = SetVelocityPacket(SetOperation.ADD, cart_id=self.cart_id, value=random_start, actobs_tracker=1)
        # self.network_manager.send_packet(velo_pkt)
        
        state = self.observe()

        self.last_observation_time = 0
        self.last_action_time = 0.0
        print(state)
        return state
        
    def step(self, action: Action) -> StepInfo:
        """
        Does a step and returns the StepInfo related to the step.
        """
        
        action_interval: Optional[float] = None

        if self.last_action_time != 0.0:
            if (action_interval := time() - self.last_action_time) < self.action_minimum_interval:
                # Action frequency too fast, blocking until min interval passed
                sleep(self.action_minimum_interval - action_interval)

            elif action_interval > self.action_maximum_interval:
                logger.warn(
                    "Action frequency too slow: %s > %s",
                    action_interval,
                    self.action_maximum_interval,
                )

        self.last_action_time = time()
        self.action_freq_ticker.tick()

        # action of 1 is to RIGHT, action of 0 is to the LEFT
        speed_increment = 500
        speed_increment *= 1 if action == Action.RIGHT else -1

        rand_numb = np.random.randint(0, 250) # 250 to allow values that will never be chosen in normal operation

        velo_pkt = SetVelocityPacket(SetOperation.ADD, cart_id=self.cart_id, value=speed_increment, actobs_tracker=rand_numb)
        self.network_manager.send_packet(velo_pkt)

        # # Used to prevent cart moving - for testing
        # velo_pkt = SetVelocityPacket(SetOperation.EQUAL, cart_id=self.cart_id, value=0, actobs_tracker=rand_numb)
        # self.network_manager.send_packet(velo_pkt)
        
        
        # self.action_counter += 1
        self.steps += 1
        self.policy_update_steps += 1

        info: StepInfo = {
            "action_interval": action_interval,
            "action_frequency": self.action_freq_ticker.measure(),
            "observation_interval": self.last_observation_interval,
        }

        return info

    def setup(self) -> None:
        """
        Called once when first initialised.

        Not called when agent is reset!
        """
        self.action_freq_ticker = FrequencyTicker()

    def set_environment(self: CartpoleAgentT, env: CartpoleEnv[CartpoleAgentT]) -> None:
        self.env = env

    def reward(self, state: ExternalState) -> float:
        """
        This function takes in a state and returns the appropriate reward.
        """
        # TODO 2/5
        x = state[0]
        rew = cast(float, np.sin(x / self.track_length * np.pi)**8)
        return rew

    def check_state(self, state: ExternalState) -> StateChecks:
        """
        This function takes in a state and returns True if the state is not failed.
        """
        x = self._state[0]
        theta = self._state[2]
        # Check if out of bounds - not actually being used yet!!
        # checks = {
        #     FailureDescriptors.POSITION_LEFT: x < self.failure_position[0],
        #     FailureDescriptors.POSITION_RIGHT: x > self.failure_position[1],
        # }

        checks = {
            FailureDescriptors.POSITION_LEFT: False,
            FailureDescriptors.POSITION_RIGHT: False,
            FailureDescriptors.ANGLE_LEFT: theta < self.failure_angle[0],
            FailureDescriptors.ANGLE_RIGHT: theta > self.failure_angle[1]
        }

        checks[FailureDescriptors.MAX_STEPS_REACHED] = self.steps >= self.max_steps
        # To prevent potential instability with CPU blocked by update:
        # checks[FailureDescriptors.POLICY_UPDATE_REACHED] = self.policy_update_steps % 8192 == 0
    
        return checks

    def observe(self) -> ExternalState:
        """
        Returns the current external state of the environment as seen by the agent.
        """
        if self._state is None:
            raise IOError("State not yet available.")
        else:
            # TODO 2/5
            # Return state as np.array
            return self._state[1:]

    # def observe_as_dict(self) -> ExternalStateMap:
    #     """
    #     Returns the current external state as a dictionary.
    #     """
    #     obs_ = self.observe()

    #     observation = cast(
    #         ExternalStateMap,
    #         {self.external_state_idx(x).name.lower(): obs_[x] for x in range(len(obs_))},
    #     )

    #     return observation

    def setup_by_environment(
        self, network_manager: NetworkManager, observation_buffer_size: int = 100
    ) -> None:
        """
        Called by environment. Initialises agent NetworkManager and observation
        buffer size.
        """
        self.network_manager = network_manager
        self.observation_buffer_size = observation_buffer_size


    def update_goal(self, goal_params: GoalParams) -> None:
        """
        Currently, sets track_length at start and does not change!
        Update with new track length after limit finding check
        """
        ## Update track length
        self.__dict__ |= goal_params

    def estimate_angular_velocity(self) -> float:
        dt = self.dTime # time when
        # print(self._state[2])
        raw = self._state[2] - self.agent_observation_buffer[-2][2] # newAngle - oldAngle
        # print(self.agent_observation_buffer[-2][2])

        if raw == 0:
            angular_speed = 0
        else:
            abs_raw = abs(raw)

            # modified from https://stackoverflow.com/a/57315426 <- incorrect sign during angle wrap around
            # assumes smaller angle is correct, first term is "modular" arithmetic in C++ (rather than remainder)
            # see https://stackoverflow.com/a/44197900]
            abs_dtheta = min((-abs_raw)%4096, abs_raw%4096)


            # hacky way of making sure direction is preserved at 360 -> 0 wrap around
            # assume that if the angle jump is greater than 2000, we have passed through zero...
            if (abs_raw > 2000):
                direction = -1*(abs_raw / raw) # since both floats, need to check not equal to zero to prevent nan/inf
            else:
                direction = (abs_raw / raw)

            angular_speed = direction * abs_dtheta / dt * 1e5
        return angular_speed


    def absorb_packet(self, packet: CartSpecificPacket) -> None:
        """
        Takes in observation, updates the agent's current state, adds state to observation buffer.
        """
        if isinstance(packet, ObservationPacket):
            # Max value of uint32_t: 4_294_967_295
            has_rolled_over = self.last_observation_time - packet.timestamp_micros > 1_000_000_000

            if has_rolled_over or packet.timestamp_micros > self.last_observation_time:
                self.env: ExperimentalCartpoleEnv

                if (
                    (observation_interval := packet.timestamp_micros - self.last_observation_time)
                    > self.observation_maximum_interval
                    and self.last_observation_time != 0
                    and self.env.environment_state["experiment_state"] != ExperimentState.ENDING
                    and self.env.environment_state["experiment_state"] != ExperimentState.RESETTING
                ):
                    logger.warn(
                        "Observation interval too long: %s > %s",
                        observation_interval,
                        self.observation_maximum_interval,
                    )

                self.last_observation_time = packet.timestamp_micros
                self.last_observation_interval = observation_interval / 1e6  # μs -> s

                x = packet.position_steps
                dx = packet.velocity
                theta = packet.angle_deg
                self.dTime = packet.dTime

                
                self._state = np.array(
                    (
                        x,
                        dx,
                        theta,
                        self.estimate_angular_velocity()
                    )
                )
                # print(self._state)
                # print(theta, self.dTime) # dt roughly 4012 with no angle offset calculation
                self.agent_observation_buffer.append(self._state)

        else:
            raise TypeError(f"Agent does not handle {type(packet)}")
    
    def is_settled(self, settle_upright: bool = False) -> bool:
        """
        Returns True if the cart is considered settled."""
        xs = [state[0] for state in self.agent_observation_buffer]
        thetas = [state[2] for state in self.agent_observation_buffer]

        if settle_upright:
            return all(
                [
                    max(xs) - min(xs) <= self.settled_x_threshold,
                    max(thetas) - min(thetas) <= self.settled_up_theta_threshold
                ]
            )
        else:
            return all(
                [
                    max(xs) - min(xs) <= self.settled_x_threshold,
                    max(thetas) - min(thetas) <= self.settled_down_theta_threshold
                ]
            )

CartpoleAgentT = TypeVar("CartpoleAgentT", bound=CartpoleAgent)
