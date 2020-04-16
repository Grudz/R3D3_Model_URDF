cmake_minimum_required(VERSION 2.8.3)
project(igvc_flatland)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  flatland_server
  flatland_plugins
  flatland_msgs
  pluginlib
  roscpp
  cmake_modules
  std_msgs
  message_generation
  visualization_msgs
  sensor_msgs
)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED)

# yaml-cpp
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

add_message_files(
  FILES
    Waypoint.msg
    WaypointList.msg
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES igvc_flatland_plugins_lib
  CATKIN_DEPENDS pluginlib roscpp flatland_server std_msgs
  DEPENDS OpenCV YAML_CPP
)

# Extract *.tar.gz files in the maps folder
FILE(GLOB files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/maps" "${CMAKE_CURRENT_SOURCE_DIR}/maps/*.tar.gz")
FOREACH(filename ${files})
  MESSAGE(STATUS "Extracting file: ${filename}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf ${filename}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/maps
  )
ENDFOREACH(filename)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_library(igvc_flatland_plugins_lib
  src/AutoJudge.cpp
  src/RandomGates.cpp
  src/final_project.cpp
)
add_dependencies(igvc_flatland_plugins_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(igvc_flatland_plugins_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

add_executable(final_project src/final_project.cpp)

add_dependencies(final_project ${PROJECT_NAME}_gencfg)

target_link_libraries(final_project
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
  )

### Install Rules
install(
  DIRECTORY
    launch
    maps
    robot
    rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES
    flatland_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS
    igvc_flatland_plugins_lib
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
