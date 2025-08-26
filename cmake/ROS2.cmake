cmake_minimum_required(VERSION 3.8)
project(seeker)

# ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# 添加包含目录
include_directories(
  include
)

# 根据架构设置库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86")
  set(LIB_DIR "libs/x86")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(LIB_DIR "libs/arm64")
endif()

link_directories(${LIB_DIR}
)

# 生成可执行文件
add_executable(seeker_node
  src/seeker_ros2.cpp
)

# 链接依赖库
target_include_directories(seeker_node
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(seeker_node
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${stereo_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}
  seeker
  usb-1.0
)

# 安装规则
install(TARGETS seeker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${LIB_DIR}/libseeker.so
  DESTINATION lib/
)

install(DIRECTORY 
  launch/ros2
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ./config/ DESTINATION share/${PROJECT_NAME}/config/)

ament_package()
