cmake_minimum_required(VERSION 2.8.3)
project(unitree_lidar_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -llz4")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG -llz4") 

find_package(PCL REQUIRED QUIET)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  rosparam
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  roslib
)
add_message_files(
  FILES
  Route.msg
  PathScore.msg
  LenWay.msg
  Route.msg
  Routes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL

  CATKIN_DEPENDS
  rosparam
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ~/unilidar_sdk/unitree_lidar_sdk/include
)
# --- C++ 코드 추가 ---
## 실행 파일 추가 (중복 해결을 위해 이름 변경)
add_executable(tf_broad_lidar src/tf_broad_lidar.cpp)

## tf_broad_lidar 실행 파일에 라이브러리 연결
target_link_libraries(tf_broad_lidar
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# C++ 노드에 필요한 라이브러리 연결
target_link_libraries(tf_broad_lidar
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

catkin_install_python(PROGRAMS
  src/newbuffer_fin.py
  src/lin_re2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


link_directories(
  lib
  ${PCL_LIBRARY_DIRS}
  ~/unilidar_sdk/unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}
)

set(EXTRA_LIBS 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  libunitree_lidar_sdk.a
)

add_executable(unitree_lidar_ros_node src/unitree_lidar_ros_node.cpp)
target_link_libraries(unitree_lidar_ros_node ${EXTRA_LIBS})
