cmake_minimum_required(VERSION 3.8)
project(conductor)
if(POLICY CMP0148) 
cmake_policy(SET CMP0148 OLD) 
endif()

## UTF-8 编码
add_compile_options(-fexec-charset=UTF-8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(PythonInterp 3.6 REQUIRED)
# find_package (Python3 COMPONENTS Interpreter Development)
find_package(nav_msgs REQUIRED)

# 获取 src/lib/ 目录下的所有源文件
file(GLOB CONDUCTOR_SOURCES "src/lib/*.cpp")

# 将获取的源文件添加到库中
add_library(Conductor ${CONDUCTOR_SOURCES})

target_include_directories(Conductor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/include  # 添加生成的包含目录
)



# 添加自定义消息

# Add executable
ament_target_dependencies(Conductor rclcpp tf2_ros mavros_msgs geometry_msgs geographic_msgs std_msgs nav_msgs)

#add_executable(test_fixed_point_ctrl test/fixed_point_ctrl_test.cpp)
#ament_target_dependencies(test_fixed_point_ctrl rclcpp std_msgs geometry_msgs mavros_msgs geographic_msgs)
#target_link_libraries(test_fixed_point_ctrl Conductor)

# add_executable(test_takeoff test/takeoff_test.cpp)
# ament_target_dependencies(test_takeoff rclcpp std_msgs geometry_msgs mavros_msgs geographic_msgs)
# target_link_libraries(test_takeoff Conductor)

add_executable(go test/go.cpp)
ament_target_dependencies(go rclcpp std_msgs geometry_msgs mavros_msgs geographic_msgs nav_msgs)
target_link_libraries(go Conductor)

add_executable(fuck_mid360 test/fuck_mid360.cpp)
ament_target_dependencies(fuck_mid360 rclcpp std_msgs geometry_msgs mavros_msgs geographic_msgs nav_msgs)
target_link_libraries(fuck_mid360 Conductor)
# Install the executable
install(TARGETS
  go
  fuck_mid360
  # test_takeoff

  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  Conductor
  DESTINATION lib
)
# install(
#   PROGRAMS
#   scripts/turtle_spawner.py
#   DESTINATION lib/${PROJECT_NAME}
# )



target_link_libraries(Conductor "${cpp_typesupport_target}")

# see here: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#link-against-the-interface

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