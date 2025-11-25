cmake_minimum_required(VERSION 3.8)
project(map_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)
add_executable(complete_map_solver src/complete_map_solver.cpp)
ament_target_dependencies(complete_map_solver rclcpp cg_interfaces)
add_executable(lobotomy_solver src/lobotomy_solver.cpp)
ament_target_dependencies(lobotomy_solver rclcpp cg_interfaces)
add_executable(mapper_solver src/mapper_solver.cpp)
ament_target_dependencies(mapper_solver rclcpp cg_interfaces)
add_executable(mapper_solver_stats src/mapper_solver_stats.cpp)
ament_target_dependencies(mapper_solver_stats rclcpp cg_interfaces)

install(TARGETS 
  complete_map_solver
  lobotomy_solver
  mapper_solver
  mapper_solver_stats
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
