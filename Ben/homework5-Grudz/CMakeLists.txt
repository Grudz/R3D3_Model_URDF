cmake_minimum_required(VERSION 2.8.3)
project(homework5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(diff_drive_mantis src/diff_drive_mantis.cpp)
target_link_libraries(diff_drive_mantis
  ${catkin_LIBRARIES}
)
add_executable(test_homework5 test/test_homework5.cpp)
target_link_libraries(test_homework5
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)
