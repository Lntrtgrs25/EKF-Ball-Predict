# Soccer MCL Simulation

A custom 2D simulation and visualization system for humanoid robot localization in robot soccer using Monte Carlo Localization (MCL) and vision-based detection.
The simulation is built on a ROS2-based architecture with PyGame and is specifically designed for the software environment of the ICHIRO ITS humanoid robot team.

This project is developed for research and development purposes for the RoboCup Humanoid Robot Soccer Team [**ICHIRO ITS**](https://github.com/ichiro-its/)

## How to run
1. cd EKF-Ball-Predict
2. source ~/ichiro-ws/install/setup.bash
3. export PYTHONPATH=$PYTHONPATH:$(pwd)
4. python3 -m soccer_mcl_sim.sim
