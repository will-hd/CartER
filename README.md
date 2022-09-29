CartER
==========

CartER is my summer project at the Biological and Soft Systems (BSS) sector of Cavendish Laboratory, University of Cambridge.

It is a continuation of the project undertaken by
[Jeppe Klitgaard](https://github.com/JeppeKlitgaard/CartER/) in 2021.

This branch is a refactored version of Jeppe's work that considers only a single cart agent. Issues with the original version can be read [here](https://jeppeklitgaard.github.io/CartER/articles/handover_v1/#what-does-not-work). The single cart version has been simplified, ideally to work seamlessly with the well-known OpenAI gym API.

The observation space also includes the rotational position for each pendulum, which is measured using magnetic rotary encoders.

The goal is to use Reinforcement Learning (RL) algorithms to accomplish certain tasks (for example, balancing or swing-up) for increasingly difficult mechanical configurations that may include springs connecting the two pendula.

An in-sillica approach is also done in Python, though the experimental part of the project is an important part and the simulated approach is only complementary.

## Proof of concept

https://user-images.githubusercontent.com/107655543/193059563-f359427b-b478-4a39-b5ee-ea20390d01b1.mp4

The folder "commander" contains a Cartpole environment to be used in the same way as the gym API.

The forked branch CartER-single includes two main files "q_learning.py" and "swingup_to_pid_demo.py". At the time of writing, the setup is configured such that the swingup_to_pid_demo.py should work out of the box. Small tweaks may be needed for "q_learning.py" to run correctly (such as ensuring the obervation size returned is expected by the alogrithm.

## Build system

CartER makes use of the PlatformIO toolchain for managing the embedded
controller code for the Arduino Due that powers the project.

The libraries used are defined in `./controller/platformio.ini` along with the
build configuration.
