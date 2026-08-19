cmake_minimum_required(VERSION 3.8)
project(maze_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveRobotX.action"
  "action/MoveRobotYaw.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)


ament_python_install_package(maze_solver_py)

install(PROGRAMS
  maze_solver_py/action_servers.py
  maze_solver_py/maze_client.py
  maze_solver_py/move_x_action_server.py
  maze_solver_py/move_yaw_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
