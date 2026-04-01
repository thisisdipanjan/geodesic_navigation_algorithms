cmake_minimum_required(VERSION 3.8)
project(uninformed_search_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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

add_library(uninformed_search_lib SHARED
  src/breadth_first_search.cpp
  src/depth_first_search.cpp
  src/dijkstra_search.cpp
  src/iterative_deepening_search.cpp
  src/uniform_cost_search.cpp
)

ament_target_dependencies(uninformed_search_lib
  rclcpp
  nav2_core
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS uninformed_search_lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()