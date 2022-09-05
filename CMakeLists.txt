cmake_minimum_required(VERSION 3.5)
project(nav2_social_mpc_controller)

set(CMAKE_BUILD_TYPE Release) # significant Ceres optimization speedup

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(obstacle_distance_msgs REQUIRED)
find_package(obstacle_distance REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

set(CMAKE_CXX_STANDARD 17)

nav2_package()

include_directories(
  #/usr/local/include  #to find lightsfm, just in case
  include
  ${CERES_INCLUDES}
)

set(dependencies
  angles
  rclcpp
  geometry_msgs
  people_msgs
  obstacle_distance_msgs
  obstacle_distance
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  #lightsfm
)

set(library_name nav2_social_mpc_controller)

add_library(${library_name} SHARED
        src/path_trajectorizer.cpp
        src/social_mpc_controller.cpp
        src/people_interface.cpp
        src/obstacle_distance_interface.cpp
)

target_link_libraries(${library_name} ${CERES_LIBRARIES})
# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   set(ament_cmake_copyright_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
#   add_subdirectory(test)
# endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core nav2_social_mpc_controller.xml)

ament_package()

