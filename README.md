CartER
==========

CartER is my summer project at the Biological and Soft Systems (BSS) sector of Cavendish Laboratory, University of Cambridge.

It is a continuation of the project undertaken by
[Jeppe Klitgaard](https://github.com/JeppeKlitgaard/CartER/) in 2021.

This branch is a refactored version of Jeppe's work that considers only a single cart agent. Issues with the original version can be read [here](https://jeppeklitgaard.github.io/CartER/articles/handover_v1/#what-does-not-work). The single cart version has been simplified, ideally to work seamlessly with the well-known OpenAI gym API.

The observation space also includes the rotational position for each pendulum, which is measured using magnetic rotary encoders.

The goal is to use Reinforcement Learning (RL) algorithms to accomplish certain physical tasks (for example, balancing or swing-up) for increasingly difficult mechanical configurations that could involve multiple pendula coupled by springs. 

## Environment and files

The folder "commander" contains a Cartpole environment to be used in the same way as the gym API. The environment can be called by importing `from commander.ml.environment import ExperimentalCartpoleEnv` and then instantiating.

For the physical system, the gym 0.21.0 API calls are equivalent: `env.step()` performs an action and returns an observation and `env.reset()` performs a reset of the hardware and waits for the system to settle.

The branch CartER-single includes two main files "q_learning.py" and "swingup_to_pid_demo.py" to be run, and no CLI is used yet as is the case in Jeppe's original work. At the time of writing, the setup is configured such that the swingup_to_pid_demo.py should work out of the box. Small tweaks may be needed for "q_learning.py" to run correctly (such as ensuring the observation size returned is expected by the algorithm).

## Proof of concept

https://user-images.githubusercontent.com/107655543/193061080-daaa69d9-861b-4e1c-b14c-a2ea745fbae2.mp4

The video demonstrates the ability of the setup to swing-up (albeit rather inefficiently!) and switch to a PID controller, using the "swingup_to_pid_demo.py" script. It should be noted that a more robust swing-up method is needed to ensure 100% reproducibility since the PID is sometimes unable to 'catch' the pole and remain stable.

Using the Q-Learning algorithm in "q_learning.py" with a reward function that favours the cart being central (with no consideration of the pole angle), such as a $sin^4(x)$ function, the system was able to learn (see demo image): 

![BOUNCE_Q_Learning_TRIAL](https://user-images.githubusercontent.com/107655543/193065077-0ce8271d-37f6-47cf-8d50-76ac16d67d90.png)


## Build system

CartER makes use of the PlatformIO toolchain for managing the embedded
controller code for the Arduino Due that powers the project.

The libraries used are defined in `./controller/platformio.ini` along with the
build configuration.
