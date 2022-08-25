from __future__ import annotations

import logging
from collections.abc import Mapping, Sequence
from time import sleep, time
from typing import Any, Generic, Optional, Type, TypedDict, cast
import numpy as np
import time
import gym
from gym import spaces
from torch import pinverse

from commander.type_aliases import ExternalState, StepInfo, StepReturn
from commander.network.network_constants import (ExperimentInfoSpecifier, SetOperation, 
DEFAULT_BAUDRATE, DEFAULT_PORT, CartID, FailureMode)
from commander.ml.ml_constants import Action, FailureDescriptors
from commander.ml.agent.agent import CartpoleAgentT, CartpoleAgent
from commander.experiment import ExperimentState

from commander.utils import FrequencyTicker, raises

from commander.network.network import NetworkManager
from commander.network.protocol import (
    CartSpecificPacket,
    CheckLimitPacket,
    DebugPacket,
    DoJigglePacket,
    ErrorPacket,
    ExperimentDonePacket,
    ExperimentInfoPacket,
    ExperimentStartPacket,
    ExperimentStopPacket,
    FindLimitsPacket,
    InfoPacket,
    NullPacket,
    ObservationPacket,
    RequestDebugInfoPacket,
    SetMaxVelocityPacket,
    SetVelocityPacket,
    SoftLimitReachedPacket,
)
import logging
logger = logging.getLogger(__name__)

class EnvironmentState(TypedDict):
    experiment_state: Optional[ExperimentState]
    position_drifts: int
    failure_mode: FailureMode
    track_length: Optional[int]
    last_observation_times: int
    available_memory: Optional[int]

class ExperimentalCartpoleEnv(gym.Env):
    """
    An environment that represents the Cartpole Environment and
    implements networking with the physical experiment.

    | Num | Action                 |
    |-----|------------------------|
    | 0   | Push cart to the left  |
    | 1   | Push cart to the right |
    
    Attributes
    ----------
    port: (str) port for arduino 
    baudrate: (int) 
    network_manager: (NetworkManager) responsible for communication with arduino
    network_reseter: (NetworkManager) used only for opening and closing to force DTR reset

    _agent: (CartpoleAgentT) Agent in the environment
    action_space: (gym.space) Discrete(2) agent can choose back and forth
    observation_space: (gym.space) for gym API
    environment_state: (dict) Describes the environment state 

    episode: (int) No. episodes (+1 each reset called)
    world_time_start: (float?) Real time when experiment was started
    world_time: (float) Real timer
    observation_freq_ticker: (FrequencyTicker()) Object to measure the frequency of actions

    steps: (int) Counts number of steps of episode, set to zero each reset.
    

    -------
    Methods
    -------
    step() - Required for gym API
        Performs step in environment, given an action.
    reset() - Required for gym API
        Resets the environment to an initial state and returns the initial observation.
    setup()
        Called when environment first initialised
    _do_rig_reset()
        Performs hardware specific resets.
    _reset_environment_state()
    _do_rig_setup()
        Performs hardware specific setup
    init_gym_spaces()
        Initialises gym observation and action spaces
    get_agent() - probably not needed rn
        Getter method
    is_settled()
        Returns True if agent considers itself settled
    wait_for_settled()
        Calls a network tick until system is settled.
    network_tick()
        Ticks network by calling to digest and _process_buffer
    _distribute_packets()
        Passes packet onto agent to absorb
    _process_buffer
        Processes packets in the internal buffer of network manager.
    end_experiment()
        If failed, ends the (hardware) experiment.
    _has_failed
        Returns if the experiment hase failed: the environment_state is not nul.
    """

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baudrate: int = DEFAULT_BAUDRATE,
    ) -> None:
        self.port = port
        self.baudrate = baudrate

        self.network_manager = NetworkManager(port=self.port, baudrate=self.baudrate)
        # This NetworkManager points to the Native USB Serial port.
        # It is only opened and closed, to trigger DTR reset on arduino when an 
        # experiment is started.
        self.network_reseter = NetworkManager(port="/dev/ttyACM0")
        
        self._agent = CartpoleAgent()
        self.failure_id = False
        self.setup()

       
    def step(self, action: Action) -> StepReturn:
        """
        Performs step in environment, given an action.
        Gym cartpole env:
        - Takes state from previous step
        - Performs action
        - Gets new state
        - Done?
        Exp cartpole env:
        - Agent performs step
        - Check if failed
        - Wait for a new observation

        Returns:
            state (np.float32)
            reward
            done (bool)
            info (dict)
        """
        self.observation_freq_ticker.tick()
        self.observation_freq_ticker.tick()

        info = {}
        dones = {}
        # logger.debug(action) 
        info["agent_stepinfo"] = self._agent.step(action)
        self.network_tick()

        # TODO 3/5
        # Check if we have failed - why here??! Checks for hw failure (e.g. soft limit)
        if self._has_failed():
            print("In _has_failed if statement")
            info["Fail"] = {
                "failure_modes": [self.environment_state["failure_mode"].describe()]
            }
            dones["softlimit_done"] = True

        # Wait for new observation
        while True and not self._has_failed():
            
            all_observations_are_new = all(
                [
                    self.environment_state["last_observation_time"]
                    != self._agent.last_observation_time
                ]
            )

            if all_observations_are_new:
                break

            self.network_tick()

               
        # Take observation of experiment state
        observation = self._agent.observe()
        print(observation)

        # print(self._agent.estimate_angular_velocity())
        # print(observation)
        # observation_dict = self._agent.observe_as_dict()
        # logger.debug(f"obs1: {observsation}")
        self.environment_state["last_observation_time"] = self._agent.last_observation_time

        # Populate step information for debugging and callbacks
        step_info: StepInfo = {}
        # step_info["x"] = observation_dict["x"]
        # step_info["dx"] = observation_dict["dx"]
        step_info["environment_episode"] = self.episode
        step_info["available_memory"] = self.environment_state["available_memory"]

        world_time = 1#time() - self.world_time_start
        step_info["world_time"] = world_time
        step_info["observation_frequency"] = self.observation_freq_ticker.measure()
        step_info["serial_in_waiting"] = self.network_manager.serial.in_waiting

        # Add step_info to info
        info["step_info"] = step_info # Previously was |= pipe-equals

        checks = self._agent.check_state(observation)

        if checks[FailureDescriptors.MAX_STEPS_REACHED] or checks[FailureDescriptors.ANGLE_LEFT] or checks[FailureDescriptors.ANGLE_RIGHT]:
            # to speed up end
            self.failure_id = True

        done = any(checks.values()) or any(dones.values())

        reward = self._agent.reward(observation) if not done else 0.0
        # reward = +1 if not done else 0.0
        # logger.debug(f"obs2: {observation}, rew: {reward}")
        self.steps += 1

        if done:
            failure_modes = [k.value for k, v in checks.items() if v]
            logger.info(f"Failure modes: {failure_modes}")

            info["failure_modes"] = failure_modes
            print(info)

            end_experiment_info = self.end_experiment()
            # deepmerge.merge_or_raise.merge(info, end_experiment_info)

            # TODO
            logger.debug("Environment state: %s", self.environment_state)

        return observation, reward, done, info

    def reset(self):
        """
        Resets the environment to an initial state and returns the initial observation.
        Returns:
            state 
                observation of initial state
            info (optional dict)
                contains auxiliary information complementing the observation

        """
        # Agent _pre_reset
        self._agent._pre_reset()
        

        self._do_rig_reset()
        self.episode += 1

        observation = self._agent.reset()
        # ## Environment-level resets
        self.steps: int = 0
        self.world_time: float = 0.0
        self.observation_freq_ticker.clear()

        self.failure_id = False
        return observation

    def setup(self):
        """
        Called when environment first initialised.

        Not called after each reset.
        """
        logger.info("Running experimental environment setup")

        self.init_gym_spaces()
        self._reset_environment_state()
        self.environment_state["experiment_state"] = ExperimentState.STARTING

        # Agent setups
        self._agent.set_environment(self)
        self._agent.setup_by_environment(
                network_manager=self.network_manager)

        self.episode: int = 0

        self.observation_freq_ticker = FrequencyTicker()
        self._do_rig_setup()
        
    def init_gym_spaces(self) -> None:
        # Calculate size of spaces.
        # Factors of 2 are to ensure that even failing observations are still within
        # the observation space.
        low = np.array(
            [
                # -1000, #self.goal_params["failure_position"][0] * 2,  # Position
                -np.finfo(np.float64).max,  # Velocity can be any float
                0, # theta
                -np.finfo(np.float64).max
            ],
            dtype=np.float64,
        )
        high = np.array(
            [
                # +27000, #self.goal_params["failure_position"][1] * 2,  # Position
                np.finfo(np.float64).max,  # Velocity can be any float
                5000, # max only 4096
                np.finfo(np.float64).max
            ],
            dtype=np.float64,
        )

         # Can only apply two actions, back or forth
        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(low, high, dtype=np.float64)

    def _do_rig_reset(self) -> None:
        """
        Runs reset process of hardware environment. Should be called during 
        gym env reset().
        - Sets max velocity
        - Set velocity
        - Ask controller to start experiment
        - Set zero angles
        - Set
        """

        logger.info("Resetting experimental environment")
        self.environment_state["experiment_state"] = ExperimentState.RESETTING
        self._reset_environment_state()

        
        self.network_manager.assert_ping_pong()

        # Set max velocity
        logger.info("Setting max velocity")
        max_velo_pkt = SetMaxVelocityPacket(SetOperation.EQUAL, cart_id=CartID.ONE, value=10_000, actobs_tracker=2)
        self.network_manager.send_packet(max_velo_pkt)

        # Set velocity
        logger.info("Setting velocity to zero")
        velo_pkt = SetVelocityPacket(SetOperation.EQUAL, cart_id=CartID.ONE, value=0, actobs_tracker=1)
        self.network_manager.send_packet(velo_pkt)

        # Ask controller to start experiment
        logger.info("Starting experiment")
        experiment_start_pkt = ExperimentStartPacket(0)
        self.network_manager.send_packet(experiment_start_pkt)
        self.network_manager.get_packet(
            ExperimentStartPacket, digest=True, block=True, callback=self._process_buffer
        )
        
        while raises(self._agent.observe, IOError):
            
            obs_pkts = self.network_manager.get_packets(
                ObservationPacket, digest=True, callback=self._process_buffer
            )
            self._distribute_packets(obs_pkts)

        self.wait_for_settled()

        # Jiggle - not implemented, change to wait/sleep timeout # TODO 3/5
        logger.info("Jiggling carts to zero angle")
        jiggle_pkt = DoJigglePacket()
        self.network_manager.send_packet(jiggle_pkt)
        self.network_manager.get_packet(
            DoJigglePacket, digest=True, block=True, callback=self._process_buffer
        )

        self.wait_for_settled()

        # Set velocity
        logger.info("Setting velocity to zero")
        velo_pkt = SetVelocityPacket(SetOperation.EQUAL, cart_id=CartID.ONE, value=0, actobs_tracker=3)
        self.network_manager.send_packet(velo_pkt)

        self.world_time_start = 1#time()

        self.environment_state["experiment_state"] = ExperimentState.RUNNING
    
    def _reset_environment_state(self) -> None:
        self.environment_state: EnvironmentState = {
            "experiment_state": None,
            "position_drift": None,
            "failure_mode": FailureMode.NUL,
            "track_length": None,
            "last_observation_time": None,
            "available_memory": None,
        }

        self.environment_state["last_observation_time"] = 0

    def _do_rig_setup(self) -> None:
        # Setup network managers
        logger.info("Opening serial connection to controller")
        self.network_manager.open()
        # DTR arduino reset
        self.network_reseter.open()
        logger.info("Opening and closing Native Serial USB port /dev/ttyACM0 to trigger DTR reset")
        self.network_reseter.close()
        
        logger.info("Reading inital output")
        initial_output = self.network_manager.read_initial_output(print_=True)
        logger.debug("Initial output: %s", initial_output)

        # Check connection
        self.network_manager.assert_ping_pong()

        # Find limits
        logger.info("Finding limits")
        find_limits_pkt = FindLimitsPacket()
        self.network_manager.send_packet(find_limits_pkt)

        # Wait for limit finding
        self.network_manager.get_packet(
            FindLimitsPacket, digest=True, block=True, callback=self._process_buffer
        )

        self.network_tick()

        logger.info("Limits found")

        # Set velocity
        logger.info("Setting velocity to zero")
        velo_pkt = SetVelocityPacket(SetOperation.EQUAL, cart_id=CartID.ONE, value=0, actobs_tracker=0)
        self.network_manager.send_packet(velo_pkt)

        debug_info_pkt = RequestDebugInfoPacket()
        self.network_manager.send_packet(debug_info_pkt)

        # Flush DebugPackets
        self.network_manager.get_packets(
            RequestDebugInfoPacket, digest=True, block=True, callback=self._process_buffer
        )

        logger.info("Experimental environment setup done")    

    def get_agent(self) -> CartpoleAgentT:
        """
        Returns cartpole agent
        """
        return self._agent
    
    def wait_for_settled(self, settle_upright: bool = False) -> None:
        """
        Calls a network tick until system is settled.
        """
        logger.debug("Waiting for system to settle")

        while not self.is_settled(settle_upright):
            self.network_tick()

        logger.debug("Experiment settled")

    def is_settled(self, settle_upright: bool = False) -> bool:
        """Returns True if agent considers itself settled"""
        return self._agent.is_settled(settle_upright)


    def network_tick(self) -> None:
        """Ticks network by calling to digest and _process_buffer"""
        self.network_manager.digest()
        self._process_buffer()
      
    def _distribute_packets(self, packets: Sequence[CartSpecificPacket]) -> None:
        """Passes packet to agent to absorb."""
        for packet in packets:
            self._agent.absorb_packet(packet)

    def _process_buffer(self) -> None:
        """
        Processes packets in the internal buffer of network manager. Call often
        (currently each step and as callback).
        """
        # NullPackets
        self.network_manager.get_packets(NullPacket, digest=False)

        # ObservationPackets
        obs_pkts = self.network_manager.get_packets(ObservationPacket, digest=False)
        self._distribute_packets(obs_pkts)

        # DebugPackets
        dbg_pkts = self.network_manager.get_packets(DebugPacket, digest=False)
        for dbg_pkt in dbg_pkts:
            logger.debug("CONTROLLER| %s", dbg_pkt.msg)

        # InfoPackets
        info_pkts = self.network_manager.get_packets(InfoPacket, digest=False)
        for info_pkt in info_pkts:
            logger.info("CONTROLLER| %s", info_pkt.msg)

        # ErrorPackets
        err_pkts = self.network_manager.get_packets(ErrorPacket, digest=False)
        for err_pkt in err_pkts:
            logger.error("CONTROLLER| %s", err_pkt.msg)

        # ExperimentInfoPackets
        exp_info_pkts = self.network_manager.get_packets(ExperimentInfoPacket, digest=False)
        for exp_info_pkt in exp_info_pkts:
            if exp_info_pkt.specifier == ExperimentInfoSpecifier.POSITION_DRIFT:

                self.environment_state["position_drift"] = cast(
                    int, exp_info_pkt.value
                )

            elif exp_info_pkt.specifier == ExperimentInfoSpecifier.FAILURE_MODE:
                sleep(1)
                self.environment_state["failure_mode"] = cast(FailureMode, exp_info_pkt.value)

            elif exp_info_pkt.specifier == ExperimentInfoSpecifier.TRACK_LENGTH_STEPS:
            # Update track_length for reward calculations
                self.environment_state["track_length"] = cast(int, exp_info_pkt.value)

                self._agent.update_goal(
                    {"track_length": cast(int, self.environment_state["track_length"])}
                )

            elif exp_info_pkt.specifier == ExperimentInfoSpecifier.AVAILABLE_MEMORY:
                self.environment_state["available_memory"] = cast(int, exp_info_pkt.value)

            else:
                raise ValueError(
                    "Environment does not know how to deal with specifier: "
                    + exp_info_pkt.specifier.name
                )

            # SoftLimitReachedPackets
            # Just ignore since we currently use ExperimentInfoPackets to determine done state
            self.network_manager.get_packets(SoftLimitReachedPacket, digest=False)

        if len(self.network_manager.packet_buffer) >= 3:
            logger.error(
                "Had packets in buffer after processing: %s", self.network_manager.packet_buffer
            )
    

    def end_experiment(self) -> dict[str, Any]:
        """
        If failed, end the (hardware) experiment.

        """
        # add: if reason for ending experiment is max_policy update or max_steps, set velocity to be
        self.environment_state["experiment_state"] = ExperimentState.ENDING

        info: dict[str, Any] = {}
        
        logger.info("Ending experiment")
        
        if self.failure_id:
            logger.info("Setting velocity to zero to speed up end")
            velo_end_pkt = SetVelocityPacket(SetOperation.EQUAL, cart_id=CartID.ONE, value=0, actobs_tracker=0)
            self.network_manager.send_packet(velo_end_pkt)

        self.wait_for_settled()

        # Check limits
        logger.info("Checking limits")
        chk_pkt = CheckLimitPacket()
        self.network_manager.send_packet(chk_pkt)
        self.network_manager.get_packet(
            CheckLimitPacket, digest=True, block=True, callback=self._process_buffer
        )

        # Stop experiment
        logger.info("Stopping carts")
        stop_pkt = ExperimentStopPacket()
        self.network_manager.send_packet(stop_pkt)

        # Wait for stop ACK
        self.network_manager.get_packet(
            ExperimentStopPacket, digest=True, block=True, callback=self._process_buffer
        )

        # Wait for experiment to end
        self.network_manager.get_packet(
            ExperimentDonePacket, digest=True, block=True, callback=self._process_buffer
        )

        # Process final packets
        self.network_tick()

        info["position_drift"] = self.environment_state["position_drift"]
        info = self.environment_state

        self.environment_state["experiment_state"] = ExperimentState.ENDED

        return info

    def _has_failed(self) -> bool:
        """Returns if the experiment hase failed: the environment_state is not nul."""
        return self.environment_state["failure_mode"] is not FailureMode.NUL

    

        