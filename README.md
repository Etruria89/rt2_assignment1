# Research Track 2 - assignment 1

#### Vrep scene

```
rt2_assignment1/
  |
  launch/         - launch files
    |
    sim.launch            - Gazebo simulation
    ros2_bridge.launch     - python scripts only launch
    sim_vrep.launch   - nodes only launch
  scripts/        - python scripts
    |
    go_to_point.py        - pyhton script controlling the robot
    user_interface.py     - command line interface
  src/            - C++ source code
    |
    position_service.cpp  - returns random position
    state_machine.cpp     - manages the FSM logic
  srv/            - custom services
    !
    Command.srv           - user interface input
    Position.srv          - goal position
    RandomPosition.srv    - random pose generator
  urdf/           - robot description for Gazebo simulation
    |
    my_robot.urdf         - mobile robot description
  pioneer_fd.ttt     - Pioneer p3dx scene
  CMakeLists.txt     - CMake file
  package.xml        - manifest
```
Package Description

This package controls a mobile robot via the 'go_to_point' behaviour. More in detail, a random goal poses is generated and the robot aligns itself towards that point. The robot, then, sets its linear speed to drive to that position and, once that goal position is reached it turns itslef to match requested orientation. Unless the robot is stopped by the used the process continues to loop by generating a new target destination and pose. In this specific case, due to the fact that the robot 'go_to_point' behaviour is here implemented as a service sever, if the user wants to stop the robot it reached the predefined target postion and, oly after, it stops.
The robot is here simulated using a Pioneer robot in the Vrep simulation environment. 

Contents Explanation

This package contains 4 nodes managing the aforementioned control of a mobile robot. 
More in detail, two nodes are implemented as python scripts

    go_to_point: (go_to_point.py) the snode managing the robot speed control depending on the goal received. This node publishes on the topics /cmd_vel the velocity of the robot and read its position by subscribing the topic '/odom'
    userinterface: (.user_interfacepy) the command line user interface, which sends the requests to start and stop the go_to_point behaviour.

While other two nodes have been implemented in C++:

    positionServer: (position_serivice.cpp) the server node generating a random pose once requested.
    stateMachine: (state_machine.cpp) the finite state machine managing the request of a new goal pose when needed, sending it as a goal to 'go_to_point' node.


Finally, the control can be applied to a robot simulated using Coppeliasim, in the scene 'pioneer_fd.ttt' including a Pioneer p3dx non-holonomic mobile robot in an empty environment.

Compiling and Running

Compilation can be carried out sourcing the ROS Noetic path and typing:

```bash
path/to/ros_ws/$ catkin_make
```

Coppeliasim is then required to run the scene 'pioneer_fd.ttt'.

Once the simulation is running all the nodes for the control of the robot can be run executing the launch file:

   sim_vrep.launch: to be used in order to launch all the nodes which will communicate with the Coppelia simulation.

Known Issues and Limitations

It has been encoutered several times after the first attempt to launch the simulation the robot keeps the velocity and it goes straight.
In this case the coppelia scene has to be restarted as well as all the ros nodes. 


Nodes:
No documentation has been here reported for the LUA script file included in the Vrep scene.
