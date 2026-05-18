cmake_minimum_required(VERSION 3.8)
project(bt_navigation)

# 设置 C++ 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 查找行为树依赖包 (注意：根据你安装的版本，可能是 behaviortree_cpp_v3 或 behaviortree_cpp)
find_package(behaviortree_cpp REQUIRED)

# 包含头文件目录
include_directories(src)

# 添加可执行文件
add_executable(bt_main
  main.cpp
  src/climbOnR1.cpp
  src/decision.cpp
  src/isHighEnough.cpp
  src/nevigation.cpp
  src/putKfs.cpp
)

# 链接依赖
ament_target_dependencies(bt_main
  rclcpp
  geometry_msgs
  nav_msgs
  behaviortree_cpp
)

# --- 安装规则 ---

# 1. 安装可执行文件
install(TARGETS bt_main
  DESTINATION lib/${PROJECT_NAME}
)

# 2. 安装行为树的 XML 配置文件 (以便在运行/发布时能够找到)
install(FILES
  mytree.xml
  include/subtree_area3.xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

#cmake .. -DPython3_EXECUTABLE=/usr/bin/python3