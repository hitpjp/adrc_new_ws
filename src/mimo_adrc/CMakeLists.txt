cmake_minimum_required(VERSION 3.8)
project(mimo_adrc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 1. 编译算法库：这里必须包含 ADRC.cpp 和 MimoADRC.cpp
# 原来你可能漏掉了 src/MimoADRC.cpp
add_library(adrc_lib 
  src/ADRC.cpp
  src/MimoADRC.cpp
)

# 显式包含 include 目录，确保头文件能被找到
target_include_directories(adrc_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 2. 编译仿真节点
add_executable(mimo_node src/mimo_node.cpp)

# 链接上面生成的库
target_link_libraries(mimo_node adrc_lib)

# 添加 ROS 2 相关依赖
ament_target_dependencies(mimo_node rclcpp std_msgs)

# 3. 安装配置
install(TARGETS mimo_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()