cmake_minimum_required(VERSION 3.8)
project(offboard_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Find the gz_transport library
find_package(gz-transport12
	#REQUIRED COMPONENTS core
	NAMES
		gz-transport12
)
find_package(gz-msgs9 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-sensors7 REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(gz-rendering7 REQUIRED)

if(gz-transport12_FOUND)
	add_compile_options(-frtti -fexceptions)

	set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

	if(GZ_TRANSPORT_VER GREATER_EQUAL 12)
		set(GZ_TRANSPORT_LIB gz-transport${GZ_TRANSPORT_VER}::core)
	endif()
endif()

# declare message files
set(msg_files
  "msg/DetectorOutput.msg"
)

# generate the messages
rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files}
  DEPENDENCIES builtin_interfaces 
  ADD_LINTER_TESTS
)

# export the message runtime dependency
ament_export_dependencies(rosidl_default_runtime)

# include cpp "include" directory
include_directories(include)

# create cpp executable
add_executable(gz_true_pos_pub src/gz_true_pos_pub.cpp)
ament_target_dependencies(gz_true_pos_pub rclcpp
                        gz-transport12 gz-sensors7 gz-rendering7 gz-msgs9 gz-math7 std_msgs geometry_msgs)                        
target_include_directories(gz_true_pos_pub PUBLIC ${Eigen3_INCLUDE_DIRS})

# install cpp executables
install(TARGETS
  gz_true_pos_pub
  DESTINATION lib/${PROJECT_NAME}
)

# install python modules
# ament_python_install_package(${PROJECT_NAME})

# install python executables
install(PROGRAMS
  scripts/observer_detector_sim.py
  scripts/observer_detector_real.py
  scripts/observer_detector_exp.py
  DESTINATION lib/${PROJECT_NAME}
)

# link against the interface
#rosidl_target_interfaces(gz_true_pos_pub
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
