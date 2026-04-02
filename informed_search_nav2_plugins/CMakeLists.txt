cmake_minimum_required(VERSION 3.8)
project(informed_search_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_library(informed_search_lib SHARED
  src/a_star_search.cpp
  src/beam_search.cpp
  src/greedy_best_search.cpp
  src/hybrid_a_star_search.cpp
  src/iterative_deepening_a_star_search.cpp
)

ament_target_dependencies(informed_search_lib
  rclcpp
  nav2_core
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS informed_search_lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
