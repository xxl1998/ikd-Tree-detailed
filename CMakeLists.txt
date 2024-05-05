cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ikd_tree)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3") 

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCL REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCL_INCLUDE_DIRS}
    ikd-Tree
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(ikd_tree_test ikd-Tree/ikd_tree_test.cpp ikd-Tree/ikd_Tree.cpp)
target_link_libraries(ikd_tree_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(qr_solve src/qr_solve.cpp)
target_link_libraries(qr_solve ${OpenCL_LIBRARIES})
