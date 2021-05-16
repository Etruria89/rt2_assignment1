
cmake_minimum_required(VERSION 3.5)
project(rt2_assignment1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/Command.srv"
    "srv/Position.srv"
    "srv/RandomPosition.srv"
)
ament_export_dependencies(rosidl_default_runtime)

add_library(random_position_service SHARED
  src/position_service.cpp)
  
add_library(finite_state_machine SHARED
  src/state_machine.cpp)

  
rclcpp_components_register_node(random_position_service
  PLUGIN "rt2_assignment1::RPS"
  EXECUTABLE comp_position_service)    
# The main can be run as standalone node while the "comp_random_service" is the component
rclcpp_components_register_node(finite_state_machine
  PLUGIN "rt2_assignment1::FSM"
  EXECUTABLE comp_state_machine)

  
ament_target_dependencies(random_position_service rclcpp rclcpp_components)
ament_target_dependencies(finite_state_machine rclcpp rclcpp_components)

  
rosidl_target_interfaces(random_position_service
	${PROJECT_NAME} "rosidl_typesupport_cpp")
	
rosidl_target_interfaces(finite_state_machine
	${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
    random_position_service finite_state_machine
    comp_position_service comp_state_machine
    DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

install(
    FILES rt2_mapping.yaml
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
