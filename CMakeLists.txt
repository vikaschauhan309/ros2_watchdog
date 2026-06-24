cmake_minimum_required(VERSION 3.8)
project(ros2_watchdog)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HealthEvent.msg"
  DEPENDENCIES builtin_interfaces
)

include_directories(include)

add_library(${PROJECT_NAME}_core
  src/topic_monitor.cpp
  src/action_dispatcher.cpp
  src/watchdog_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rclcpp_lifecycle
  std_srvs
  diagnostic_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_core "${cpp_typesupport_target}")

add_executable(watchdog_node src/watchdog_node_main.cpp)
target_link_libraries(watchdog_node ${PROJECT_NAME}_core)
ament_target_dependencies(watchdog_node rclcpp rclcpp_lifecycle)

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS watchdog_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/demo_failure_sim.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config scripts tools media
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Per-file copyright boilerplate isn't this project's convention -- a single
  # LICENSE file at the repo root covers it. cpplint is reconfigured below
  # instead of excluded, since its other checks are still worth running.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-legal/copyright")

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_topic_monitor test/test_topic_monitor.cpp)
  target_link_libraries(test_topic_monitor ${PROJECT_NAME}_core)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_watchdog_lifecycle.py
    TARGET test_watchdog_lifecycle
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp rclcpp_lifecycle std_srvs diagnostic_msgs rosidl_default_runtime)

ament_package()
