# Research Track 2 - assignment 1

#### Action Task

```
rt2_assignment1/
  |
  action/         - action files
    |
    Control2_1.action           - goal and monitoring action
  doc/            - documentation
  launch/         - launch files
    |
    sim.launch            - launch file for simulation and nodes
  scripts/        - python scripts
    |
    go_to_point.py        - pyhton script controlling the robot
    user_interface.py     - command line user interface
  src/            - C++ source code
    |
    position_service.cpp  - random position service
    state_machine.cpp     - finite state machine  
  srv/            - custom services
    !
    Command.srv           - user interface input
    RandomPosition.srv    - random position service
  urdf/           - robot description for Gazebo simulation
    |
    my_robot.urdf         - mobile robot description
  CMakeLists.txt  - CMake file
  package.xml     - manifest
```
## Package Description

This package controls a mobile non-holonomic robot with a 'go_to_point' logic, 
1. a random goal is issued (a _pose_, [x,y,theta]);
2. the robot orients itself towards the [x,y] destination;
3. it then drives straight to that position (adjusting the orientation if need be);
4. having reached the [x,y] goal position the robot turns in place in order to match the goal _theta_;
5. if the user does not stop the robot GOTO step 1, otherwise stay still until asked to start again, then GOTO step 1;

Since the user request is here implemented as an action it can be preempted, stoppinng the robot at any time and then restarting it when issuing a new goal.

## Content Explanation

Two nodes are implemented as python scripts
- **go_to_point.py**: the action server managing the robot speed control depending on the goal received.
- **user_interface.py**:  the simple command line user interface, which sends the requests to start/stop the go_to_point behaviour.

Whilst the last two are C++ based nodes
- **position_service.cpp**: the server generating a random pose [x,y,theta] as a response to a request.
- **state_machine.cpp**:  the FSM managing the request of a new goal pose when needed, sending it as a goal to 'go_to_point' action server.

## Compiling

Compilation can be carried out as always with
```bash
path/to/ros_ws/$ catkin_make
```

## Running

The launch file is provided:
- **sim.launch**: to be used in order to launch all the nodes and the Gazebo simulation.
```bash
path/to/ros_ws/$ roslaunch rt2_assignment1 sim.launch
```

## Implementation Details

### StateMachine

The only choice worth of note probably regards the fact that the current robot state can be changed by either the user's input (1: start, -1: stop) or the action reaching its goal (2: action ended): in the latter case the state of the goal objective is retrieved, and a check is made on whether the action was succesful or not. If it succeeded then it starts again by defining a new random goal point, otherwise the robot will stop and wait for new user inputs.

## Requirements

In order to run this package and visualize the rodot control ligic a running istance of **Gazebo** is needed.
The definition of both the robot and the envront can be retrieved in the folder 'urdf' of this package and in the header of the *sim.launch* file.
