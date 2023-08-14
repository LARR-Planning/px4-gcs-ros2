cmake_minimum_required(VERSION 3.5)
project(px4_gcs_interfaces)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files "srv/InitHome.srv" "srv/KeyboardInput.srv" "srv/SwitchMode.srv")

rosidl_generate_interfaces(${PROJECT_NAME} ${srv_files}
	DEPENDENCIES std_msgs
)


ament_package()
