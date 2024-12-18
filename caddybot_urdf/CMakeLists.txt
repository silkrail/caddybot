cmake_minimum_required(VERSION 2.8.3)

project(caddybot_urdf)

find_package(catkin REQUIRED COMPONENTS
  urdf
  turtlebot3_description
)
catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
