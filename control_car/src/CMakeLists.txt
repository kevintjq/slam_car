# CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(control_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

# 添加可执行文件
add_executable(control_car_node control_car_node.cpp)

# 指定依赖关系
ament_target_dependencies(control_car_node
  rclcpp
  geometry_msgs
  serial
)

# 安装可执行文件
install(TARGETS
  control_car_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
