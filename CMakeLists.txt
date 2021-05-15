cmake_minimum_required(VERSION 3.5)
project(manual_composition_package)

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
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include)

add_library(manual_talker_component SHARED
  src/publisher.cpp)
  
add_library(manual_listener_component SHARED
  src/subscriber.cpp)

add_library(manual_server_component SHARED
  src/server.cpp)

add_library(manual_client_component SHARED
  src/client.cpp)

  
rclcpp_components_register_node(manual_talker_component
  PLUGIN "manual_composition_package::MinimalPublisher"
  EXECUTABLE comp_publisher)
  
rclcpp_components_register_node(manual_listener_component
  PLUGIN "manual_composition_package::MinimalSubscriber"
  EXECUTABLE comp_subscriber)
  
rclcpp_components_register_node(manual_server_component
  PLUGIN "manual_composition_package::MinimalServer"
  EXECUTABLE comp_server)
  
rclcpp_components_register_node(manual_client_component
  PLUGIN "manual_composition_package::MinimalClient"
  EXECUTABLE comp_client)
  
ament_target_dependencies(manual_talker_component rclcpp rclcpp_components std_msgs)
ament_target_dependencies(manual_listener_component rclcpp rclcpp_components std_msgs)
ament_target_dependencies(manual_server_component rclcpp rclcpp_components example_interfaces)
ament_target_dependencies(manual_client_component rclcpp rclcpp_components example_interfaces)

add_executable(manual_composition
  src/m_composition.cpp)
  
target_link_libraries(manual_composition
  manual_talker_component
  manual_listener_component
  manual_client_component
  manual_server_component)
  
ament_target_dependencies(manual_composition rclcpp rclcpp_components std_msgs example_interfaces)

install(TARGETS
   manual_talker_component
   manual_listener_component
   manual_server_component
   manual_client_component
   manual_composition
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib)
  
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
