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

