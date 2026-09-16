cmake_minimum_required(VERSION 2.8.3)

project(go2_description)

######################
### 依赖包和 Catkin 配置
######################
find_package(catkin REQUIRED COMPONENTS roslaunch robot_state_publisher rviz joint_state_publisher)

catkin_package()

# Robot description resources are installed package-relatively so launch files
# can resolve them through $(find go2_description).
foreach(dir config launch meshes urdf xacro)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
		OPTIONAL
		PATTERN "__pycache__" EXCLUDE
		PATTERN "*.pyc" EXCLUDE)
endforeach(dir)
