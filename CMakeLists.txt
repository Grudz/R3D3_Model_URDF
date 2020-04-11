cmake_minimum_required(VERSION 2.8.3)

project(r3d3_model)

# List other catkin package dependencies
find_package(catkin REQUIRED COMPONENTS
    roscpp
)

# Include core ROS library directories
include_directories(${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package()

# Compile an executable node
add_executable(r3d3_joint_pub src/r3d3JointPub.cpp src/r3d3JointPub.h src/main_r3d3_joint_pub.cpp)
target_link_libraries(r3d3_joint_pub ${catkin_LIBRARIES})

