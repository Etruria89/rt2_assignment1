# Research Track 2 - assignment 1

# Ros1 bridge task

```
rt2_assignment1/
  |
  launch/         - launch files
    |
    ros2_bridge_run.py            - nodes launcher
  src/            - C++ source code
    |
    position_service.cpp  - random position service definition 
    state_machine.cpp     - finite state machine 
  srv/            - custom services
    |
    Command.srv           - user interface service
    Position.srv          - goal position service
    RandomPosition.srv    - random position service
  doc/            - documentation
  CMakeLists.txt  - CMake file
  ros2_bash.sh  - bash script to runthe simulation with all the required nodes
  mapping_rules.yaml  - ros1_bridge mapping file
  package.xml     - manifest
  README.md       

```
## Content Description

The scripts presented in this branch of the repository show an example of a ros2 package for the partial control of a non-holonomic mobile robot.
The nodes here descibed, in fact, are not sufficient for a complete definition of the robot motion and they do not include any simulations.
For these reasons, this ros2 package is intented to be used with a ros1 simulation and a dedicated comunication package only. More in detail, this branch is supposed to be integrated with a compliant ROS1 environment including a Gazebo simulation of a non-holonomic robot in which the nodes implementing the '/go_to_point' behaviour have been predefined as presented in the **main**.
The connection between the ros1 simulation and the here presented ros2 packages is realized using the `ros1_bridge` package that must be installed and compiled. 
Once that both the ros1 simulation, the `ros1_bridge` package and the here presented package are running, via a command line interface the user can choose to make the robot start moving or stop it. Once that the robot starts moving it will randomly generate a target position and pose, it will drive towards it, and it will then repeat the process until being told to stop. Due to the fact that the robot movement is implemented using services only the user's inputs is evaluated only once the robot reached its current goal; for this reason, it cannot be stopped only in a goal pose.

In particular, the nodes described in this package are:

- **position_service.cpp:** tha implements the server for the retrieval of a random (x,y,theta) target pose;
- **state_machine.cpp:** responsible for the communication with the '/go_to_point' service when the user requires the robot to move.

## Requirements

As mentioned in the description, this package is intended to be run together with a ROS1 package containing the definition of botgh the nodes implementing a '/go_to_point' service and the same services here defined. In this case, the ros1 package must have the same name of this package and the omonym services must have exactly the same definiton. 
These conditions are needed to ensure the correct mapping between the ros1 and the ros2 environment as defined in the 'mapping_rules.yaml' file.
Moreover, it is required to have a 'ros1_bridge' package installed and compiled (see ![ros1_bridge](https://github.com/ros2/ros1_bridge) for further details).
To launch the nodes in the proper sequence a 'ros2_bash.sh' bash script has been included and can be directly executed from the folder in which repository is cloned.

Both ROS Noetic and ROS2 Foxy must be present on the system and they are sourced autonomously by the scripts. It is noteworthy that the 'ros2_bash.sh' has to be modified to 
ensure the correct path definitions.

## Compiling 

This package can be compiled as follows from the ros2 workspace directory

```bash
.../my_ros2_ws# colcon build --packages-select rt2_assignment1
```

## Running

In order to make this package communicate with the ROS1 three steps are required:

1. Launch the simulation and the needed nodes from the ROS1 in a shell with ROS1 sourced:

```bash
.../my_ros_ws/src/rt2_assignment1# roslaunch rt2_assignment1 ros2_sim.launch
```
Note: The main branch has to be used for the simulation in ROS1

2. Run the ros1_bridge package in a teerminal with both ROS1 and ROS2 sourced:
```bash
.../my_ros2_ws/src/rt2_assignment1$ ros2 run ros1_bridge dynamic_bridge
```

3. Launch the container with the components implemented in this package in a shell with ROS2 sourced:
```bash
.../ros2_ws/src/rt2_assignment1$ ros2 launch rt2_assignment1 ros2_bridge_run.py
```

Note: The two components can be run as individual nodes via:

```bash
.../my_ros2_ws# ros2 run rt2_assignment1 comp_position_service
.../my_ros2_ws# ros2 run rt2_assignment1 comp_state_machine
```

These three steps are included in the bash script `bridge_launch.sh` that can be run to have the simulation running:
```bash
.../my_ros2_ws/src/rt2_assignment1$ ./ros2_bash.sh
```
