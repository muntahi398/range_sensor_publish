cmake_minimum_required(VERSION 2.8.3)
project(sensor_pub)


find_package(catkin REQUIRED COMPONENTS
  rospy
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_pub
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




