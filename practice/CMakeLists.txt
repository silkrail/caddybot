cmake_minimum_required(VERSION 3.0.2)
project(practice)


find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  tf
  tf2
  tf2_ros
  nav_msgs 
  geometry_msgs 
  sensor_msgs 
  std_msgs
  gazebo_ros
  message_generation
  gazebo_plugins
  gazebo_ros
  caddybot_urdf
  serial
)

find_package(Eigen3 REQUIRED)

catkin_python_setup()


add_message_files(
  FILES coor.msg msgTutorial.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES practice
  CATKIN_DEPENDS roscpp gazebo_ros rospy tf tf2 tf2_ros nav_msgs geometry_msgs sensor_msgs std_msgs message_runtime serial
#  DEPENDS system_lib
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# --- C++ 코드 추가 ---
## 실행 파일 추가
add_executable(tf_broad nodes/tf_broad.cpp)
catkin_install_python(PROGRAMS
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# C++ 노드에 필요한 라이브러리 연결
target_link_libraries(tf_broad
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)


catkin_install_python(PROGRAMS
  nodes/AGV_teleop_key.py
  nodes/caddy_teleop_key.py
  nodes/imu_filter.py
  nodes/imu_filter2.py
  nodes/practice_node.py
  nodes/publisher.py
  nodes/publisher2.py
  nodes/RGintoPC.py
  nodes/vel2odom.py
  nodes/imu_log.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
