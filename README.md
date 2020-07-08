# Rotary Inverted Pendulum with DDPG
The files contained herein create a deep deterministic policy gradient (DDPG) agent.  The chosen environment is the rotary inverted pendulum platform, specifically the QUBE-Servo 2 from Quanser, built using SimMechanics by MATLAB.
The following sections describe the model and how to use it to train a DDPG agent.

## Environment
The Simulink environment is contained within the *rlCartPole.slx* file.  Opening this model builds the environment using parameters contained in the *qubeInit.m* file.  To modify the conditions, such as link length, link mass, joint damping, or even gravity, these changes can be made in the *qubeInit.m* file, or within the script prior to simulation.
It should be noted that loading *rlCartPole.slx* automatically calls *qubeInit*, so there is no need to run *qubeInit* separately.
This particular environment has four observations, the rotary arm angle, rotary arm angular velocity, pendulum arm angle, and pendulum arm angular velocity.  The actions are dependent upon the chosen controller (see below).
Within the *cartPoleScript.m* file, the first section defines the environment.  It does this by opening the model, setting the appropriate controller, then builds a Simulink reinforcement learning environment.

## DDPG Agent
The DDPG algorithm is an actor-critic model-free algorithm.  It was chosen for this application due to its ability to provide a continuous action space.  The second section within the *cartPoleScript.m* file builds the critic network, actor network, and builds the agent.

## Controller
The *rlCartPole.slx* has a variant subsystem within the Controller block which specifies what the agent output will do.  Setting `VSS_CONTROL=0` sets the controller to a single-output DDPG agent where the agent directly controls the voltages to the DC motor.  If this is the intention, make sure that `actInfo` is set properly (e.g., `actInfo = rlNumericSpec(1)`).  To use a controller that acts as a state feedback controller, set `VSS_CONTROL=1` and `actInfo=rlNumericSpec([4 1])`.  In this scenario, the DDPG agent produces 4 gains to be applied to the individual observations that are combined to a voltage signal to the DC motor.
