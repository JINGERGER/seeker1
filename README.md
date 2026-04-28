# Orin 分支 — Jetson AGX Orin 平台优化

> **当前分支**：`orin` | 平台：NVIDIA Jetson AGX Orin · L4T 36.x · CUDA 12.x · ROS2 Humble

本分支在 ROS2 Humble 支持的基础上，针对 **Jetson AGX Orin** 平台进行了深度 GPU 优化，主要贡献：

- ✅ **GPU JPEG 解码**：使用 nvJPEG 替换 CPU `cv::imdecode`，异步解码队列消除回调阻塞
- ✅ **GPU 去畸变**：将 Python `cv2.remap` 替换为 C++ `cv::cuda::remap`，消除跨进程传输和 Python 节点全部开销
- ✅ **减少 cudaMemcpy**：GPU → Image 一次拷贝（原来需经过 cv::Mat 中转两次）
- ✅ **CPU 占用大幅下降**：~75% → **~20%**，帧率从 ~16 Hz 提升至 **~20 Hz**，延迟从 ~1s 降至 **<50ms**
- ✅ **硬件自检脚本**：`script/seeker_hw_self_check.sh` 一键验收整条 ROS2 管线
- ✅ **图像去畸变**：实现了 MEI (Mei Unified Omnidirectional) 相机模型的正确去畸变算法
- ✅ **压缩图像输出**：去畸变后以 JPEG 压缩发布，带宽从 ~18 MB/s 降至 ~1-2 MB/s
- ✅ **NumPy 2.x 兼容**：移除 cv_bridge 依赖，直接用 numpy 做 ROS↔OpenCV 转换
- ✅ **视差转深度**：支持将视差图转换为度量深度图，带运行时开关
- ✅ **最小化默认话题**：所有非必要话题（视差/深度/IMU/原始鱼眼）默认关闭，按需开启

### Orin 平台环境要求

- **NVIDIA Jetson AGX Orin**（或同系列）
- **L4T 36.x**（Ubuntu 22.04）
- **ROS2 Humble**
- **CUDA 12.x**（L4T 自带）
- `sudo apt install libyaml-cpp-dev`

## ROS2 快速开始

```bash
# 编译
cd ~/ros2_ws
colcon build --packages-select seeker --symlink-install

# 启动（基础模式，默认只输出去畸变压缩图像）
ros2 launch seeker 1seeker.launch.py

# 自定义视场（推荐 1.5-2.5，越大视野越宽）
ros2 launch seeker 1seeker.launch.py undistort_fov_scale:=2.0

# 同时发布未压缩原始图像
ros2 launch seeker 1seeker.launch.py undistort_pub_raw:=true

# 启用所有话题
ros2 launch seeker 1seeker.launch.py pub_fisheye_raw:=true pub_disparity_img:=true pub_disparity:=true pub_imu:=true

# 运行时启用深度图输出
ros2 param set /disparity_to_depth enabled true
```

**参数说明：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_undistort` | `true` | 是否启用去畸变节点 |
| `undistort_fov_scale` | `1.5` | 视场缩放（>1.0 视野变宽） |
| `undistort_jpeg_quality` | `80` | 压缩图像 JPEG 质量（1-100） |
| `undistort_pub_raw` | `false` | 是否同时发布未压缩原始图像 |
| `pub_fisheye_raw` | `false` | 是否发布原始鱼眼图像和 `/all/compressed` |
| `pub_disparity_img` | `false` | 是否发布视差原始图像 |
| `pub_disparity` | `false` | 是否发布视差消息 |
| `pub_imu` | `false` | 是否发布 IMU 数据 |
| `use_depth` | `false` | 是否启动时就启用深度图转换 |

**默认输出话题（最小模式）：**
- `/fisheye_rect/{left,right,bright,bleft}/image_raw/compressed` - 去畸变 JPEG 压缩图像 ⭐

**可选话题（按需启用）：**
- `/fisheye_rect/{left,right,bright,bleft}/image_raw` - 去畸变未压缩图像（`undistort_pub_raw:=true`）
- `/fisheye/{left,right,bright,bleft}/image_raw` - 原始鱼眼图像（`pub_fisheye_raw:=true`）
- `/{front,right,back,left}/disparity` - 视差消息（`pub_disparity:=true`）
- `/{front,right,back,left}/disparity/image_raw` - 视差图像（`pub_disparity_img:=true`）
- `/{front,right,back,left}/depth/image_raw` - 深度图（`use_depth:=true`）
- `/imu_data_raw` - IMU 数据（`pub_imu:=true`）

## 硬件自检

```bash
cd ~/ros2_ws/src/seeker1
chmod +x script/seeker_hw_self_check.sh
./script/seeker_hw_self_check.sh
```

脚本会自动检查 USB 识别、节点启动、关键话题是否存在及帧率，最终给出 `PASS/WARN/FAIL` 汇总。详见 [doc/HW_SELF_CHECK.md](./doc/HW_SELF_CHECK.md)。

GPU 优化细节参见 [doc/gpu_jpeg_decode_optimization.md](./doc/gpu_jpeg_decode_optimization.md)。

> 📄 完整流水线说明、标定方法和故障排查请参阅 [ROS2_SEEKER.md](./ROS2_SEEKER.md)

---





## 售后支持请添加微信

微信号：ShiYuan_Seeker

需要技术支持麻烦添加下客服微信，淘宝等平台不允许提供第三方联系方式，请不要再淘宝问技术支持相关问题，一律不回也不做任何技术支持

![](https://img.shields.io/badge/ROS-Noetic-brightgreen)  
![](https://img.shields.io/badge/LICENSE-MIT-green.svg)

深圳市视元智能科技有限公司 seeker系列产品ROS SDK

为Seeker系列摄像头设计的开发套件，支持ROS1 Noetic环境下的多模态数据获取与处理。

## 📦 安装指南

### 环境要求

+ **Ubuntu 20.04**
+ **ROS Noetic** ([安装教程](http://wiki.ros.org/noetic/Installation/Ubuntu))
+ sudo apt install libusb-1.0-0-dev
+ pip3 install pyusb numpy ruamel.yaml

### 安装步骤

1. **创建工作空间**：

mkdir -p ~/catkin_ws/src

2. **克隆项目仓库**：

cd ~/catkin_ws/src && git clone http://gitee.com/nochain/seeker1.git

增加视差转深度图代码

git clone [https://github.com/skohlbr/disparity_image_proc.git](https://github.com/skohlbr/disparity_image_proc.git)

3. **编译项目**：

catkin_make -DCMAKE_BUILD_TYPE=Release

4. **设置环境变量**

source ~/catkin_ws/devel/setup.bash

或者：echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

5. **安装解畸变包**

dpkg -i  ~/catkin_ws/src/seeker1/deb/ros-noetic-image-undistort_0.0.0-0focal_amd64.deb

## 🔌 硬件连接

1. 使用USB3.0 Type-C线连接设备
2. 设置设备权限 sudo vim /etc/udev/rules.d/99-seeker.rules，增加udev规则

```plain
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0000", MODE="0666"
```

‌**重新加载udev规则**‌

```bash
sudo udevadm control --reload && sudo udevadm trigger
```

**然后拔插一下设备**

## 🚀 快速开始

基础数据流（鱼眼图像+IMU+视差图）

```bash
roslaunch seeker 1seeker_nodelet.launch
```

## 其他例程运行前准备：（标定参数读取）

确保模块可以正常使用，usb访问没有权限问题，执行以下命令，从模块读取标定参数并生成配置文件：

```bash
python3 ~/catkin_ws/src/seeker/scripts/1get_kalibr_info.py
```

+ ‌**输出文件路径**：生成的标定参数文件默认保存至`/tmp/kalibr_cam_chain.yaml`

### 3.3.3. 配置文件部署‌

将生成的标定文件复制到目标配置目录：

```plain
cp /tmp/kalibr_cam_chain.yaml ~/catkin_ws/src/seeker/config/seeker_omni_depth/
```

+ ‌**依赖关系**：后续拼接、解畸变等流程需基于此文件进行参数初始化‌。

## 📂 Launch文件说明

### 图像查看

```bash
# 启动基础数据流（鱼眼图像 + 视差图 + IMU）
roslaunch seeker 1seeker_nodelet.launch  
```

**rqt_gui 导入**

rosrun rqt_gui rqt_gui然后在perspectives里面点击import一个一个导入 ~/catkin_ws/src/seeker/gui的全部gui文件。然后perspectives选择seeker。查看原始图像和视差图。

![img](doc/img/image0.jpg)

### 全景拼接

**运行**

```plain
roslaunch seeker 2concat_nodelet.launch concat_method:=3
注意，开启后需要隔一会（几秒钟）才会出图
话题为 /concat_undistort_node/image
可以用rqt_image_view进行查看
```

**效果**

concat_method:=0

简单拼接的360全景图

![img](doc/img/concat_360.jpeg)

concat_method:=1 融合拼接BEV图

(blend_weight) 融合权重值，调整范围(从0.7调到0）(bev_ground_height)地面相对于模块的高度，例如(-1.0米)设置为(-1.0)(bev_fx) 虚拟摄像头焦距

![img](doc/img/concat_bev.jpeg)

concat_method:=2融合拼接360全景图

![img](doc/img/concat_3602.jpeg) 

(blend_weight) 融合权重值，调整范围(从0.7调到0）(pano_radius) 半径大小

concat_method:=3融合拼接720全景图

![img](doc/img/concat_720.jpeg) 

(blend_weight) 融合权重值，调整范围(从0.7调到0）(pano_radius) 半径大小


### 图像解畸变

**运行**

```bash
roslaunch seeker 3undistort_nodelet.launch
```

**rqt_gui查看**

rosrun rqt_gui rqt_gui然后perspectives选择undistort。查看解畸变后的图。

**效果**![img](doc/img/undistort.jpeg)

### 深度图

```plain
roslaunch seeker 4depth_image.launch
```

然后可以查看下面四个话题：

| /front/depth/image_raw |
| --- |
| /right/depth/image_raw |
| /back/depth/image_raw |
| /left/depth/image_raw |


![img](doc/img/depth.jpeg)

### 点云

```plain
roslaunch seeker 5point_cloud.launch
```

用rviz进行查看：

若要查看话题/front/points2，Fixed Frame调整为/depth0

若要查看话题/right/points2，Fixed Frame调整为/depth1

若要查看话题/back/points2，Fixed Frame调整为/depth2

若要查看话题/left/points2，Fixed Frame调整为/depth3

![img](doc/img/pointcloud.jpeg)

> _@深圳市视元智能科技有限公司_
>

## 售后支持请添加微信

微信号：ShiYuan_Seeker

