cmake_minimum_required(VERSION 2.8.3)
project(imu_integration)

# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

# sophus

find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
)
# find_package(sophus REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PROJECT_SOURCE_DIR}/thirdparty/sophus
)

add_executable(${PROJECT_NAME}_node src/main.cpp)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
