cmake_minimum_required(VERSION 3.5)
project(px4_gcs_ros2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/InitHome.srv"
	"srv/KeyboardInput.srv"
	"srv/SwitchMode.srv"
)

add_library(px4_gcs_wrapper src/px4_gcs_ros2/px4_gcs_wrapper.cc)
ament_target_dependencies(px4_gcs_wrapper rclcpp geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros)

target_include_directories(
		px4_gcs_wrapper PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>)

install(FILES include/${PROJECT_NAME}/px4_gcs_wrapper.h
		DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/px4_gcs_wrapper)

add_executable(px4_gcs_ros2_node src/px4_gcs_ros2/px4_gcs_wrapper_node.cc)
target_link_libraries(px4_gcs_ros2_node px4_gcs_wrapper)
install(TARGETS px4_gcs_ros2_node DESTINATION lib/${PROJECT_NAME})


install (DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
