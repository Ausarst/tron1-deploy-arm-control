cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(armeepose)

# 1) 找 catkin 以及我们需要的 ROS 组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  airbot_sdk
)

# 2) 第三方依赖，找 KDL（用于 RPY -> 四元数转换）
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)


# 3) 头文件路径
include_directories(
   ${catkin_INCLUDE_DIRS}
   ${orocos_kdl_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIRS}
#   ${AIRBOT_INCLUDE_DIR}
#   /usr/include/eigen3
)

# 让链接器能在 devel/lib 和源码内置 lib 目录找到 libairbot_play.so
link_directories(
  ${CATKIN_DEVEL_PREFIX}/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../airbot-sdk-2.9/lib
)

# 4) 导出 catkin 包（本包不导出消息/动态库，仅可执行程序）
catkin_package()

# 5) 源码与可执行文件
add_executable(arm_eepose_bridge
  src/ArmEeposeBridge.cpp
)

# 确保先复制 SDK 的 include 和 lib 到 devel，再来编译/链接本可执行文件
add_dependencies(arm_eepose_bridge
  airbot_sdk_include_target
  airbot_sdk_target
)

# 6) 目标链接依赖
target_link_libraries(arm_eepose_bridge
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  airbot_play
)

# 7) 安装（可选，但推荐，便于 rosrun/roslaunch 找到）
install(TARGETS arm_eepose_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 8) 安装 launch 文件，方便后续 roslaunch 查找
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
