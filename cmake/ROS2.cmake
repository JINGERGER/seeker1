cmake_minimum_required(VERSION 3.8)
project(seeker)

# ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(CUDA REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML yaml-cpp)

set(NVJPEG_INCLUDE_DIR /usr/local/cuda/include)
set(NVJPEG_LIB_DIR /usr/local/cuda/targets/aarch64-linux/lib)

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
    /usr/local/include/opencv4
    ${NVJPEG_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
)

link_directories(${NVJPEG_LIB_DIR} /usr/local/lib)

ament_target_dependencies(seeker_node
  rclcpp
  sensor_msgs
  stereo_msgs
  cv_bridge
  image_transport
  ament_index_cpp
)

target_link_libraries(seeker_node
  ${OpenCV_LIBRARIES}
  opencv_cudawarping
  opencv_imgcodecs
  seeker
  usb-1.0
  ${CUDA_LIBRARIES}
  nvjpeg
  ${YAML_LIBRARIES}
)

# 安装规则
install(TARGETS seeker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${LIB_DIR}/libseeker.so
  DESTINATION lib/
)

# 安装Python脚本
install(PROGRAMS
    script/undistort_node.py
    script/omni_undistort_node.py
    script/disparity_to_depth.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch/ros2
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

ament_package()
