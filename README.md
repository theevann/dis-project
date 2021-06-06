# DIS Project

In this repository, you will find the code for the project of Distributed Intelligent Systems. It goes with the report of the project: [LINK]

In this project, there are 3 tasks:
- localization of a robot on a trajectory, using accelerometer, wheel encoders, Kalman filters with accelerometer as process input, or Kalman filter with wheel velocity as process input. 
- Flocking of a group of robots
- Formation control of a group of robots

These tasks are to be completed in various worlds, read more in the next sections on which world corresponds to which task.

## How to run the code

1. Open Webots
2. Choose a world according to which task you want to run
3. Set the parameters of the run you want in const.h ( see corresponding section for details )
4. Launch simulation with Webots

## Worlds

World 0 is for localization, it is an empty world with a lot of space for the trajectories.
World 1 is for flocking or formation, it is a world with obstacles of different sizes, disseminated through the world. There is one group of 5 robots.
World 2 is for flocking or formation, it is a world with 2 starting areas for 2 groups of 5 robots. Here the robots have to cross each other's group.

## Localization

For the task of localization, the corresponding world is 0, and the task is 0. You can choose different preprogrammed trajectories by setting trajectory to 1 or 2. 
you should set in const.h: 
- `#define WORLD 0`
- `#define TASK 0`

The metrics are computed for the time of the trajectory, so until 115 seconds for the first trajectory and 107 seconds for the second. The final metric will be printed out to the console, and will be saved in **localization.txt** file in `metrics/`.

## Flocking 

Flocking is the task 1. It can be run in world 1 or 2. You can set the migration goal ( a position on the map ) to a different value than the preprogrammed one, in const.c, in the MIGR variable
You should set in const.h:
- `#define WORLD 1` or `#define WORLD 2`
- `#define TASK 1`
You can set in const.c: 
- `{{x, y, theta}}` for the different worlds for the MIGR variable. 

## Formation

Formation is task 2. It can be run in world 1 or 2. You can set the migration goal ( a position on the map ) to a different value than the preprogrammed one, in const.c, in the MIGR variable. You can also set the relative positions of the robots to different ones, to change the formation
You should set in const.h:
- `#define WORLD 1` or `#define WORLD 2`
- `#define TASK 2`
You can set in const.c: 
- `{{x, y, theta}}` in the different worlds for the MIGR variable. 
- `{{x, y, theta}, ...}` in the variable FORM_REL_POS, it should be of size [number of robots][3].

## PSO Optimization

You can use PSO Optimization in world 1 or 2 in order to optimize the Reynolds weights for the flocking task, or the parameters of the PID direction and velocity controller for the formation task. In order to launch a PSO optimization, you should set in const.h:
- `#define PSO true`
You can change the parameters of PSO in the parameters of the optimization algorithm from lines 15 to 24 in const.h.