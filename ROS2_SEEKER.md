# Seeker ROS2 Humble Pipeline

> 适用于 **seeker1_omni_depth** — 4路 MEI 鱼眼相机 + 4路深度/视差传感器 + IMU  
> 环境：Ubuntu 22.04 / ROS2 Humble / Python 3.10 / NumPy 2.x

---

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                      Seeker 硬件设备                             │
│   4x MEI 鱼眼相机 (1280x1088)   4x 视差传感器   1x IMU          │
└──────────────────────┬──────────────────────────────────────────┘
                       │ USB 3.0 (ID: 2207:0000)
                       ▼
┌──────────────────────────────────────────────────────────────────┐
│                   seeker_node (C++)                               │
│  • 读取 USB 数据流（MJPEG / 深度 / IMU）                          │
│  • 解码并发布 /fisheye/*/image_raw（内部，供去畸变节点使用）        │
│  • 可选发布 /*/disparity, /*/disparity/image_raw, /imu_data_raw  │
└────────────┬──────────────────────────────┬──────────────────────┘
             │ /fisheye/*/image_raw          │ /*/disparity (可选)
             ▼                              ▼
┌────────────────────────┐    ┌───────────────────────────────────┐
│  omni_undistort_node   │    │       disparity_to_depth          │
│  (Python, MEI 模型)    │    │       (Python, 可选启用)           │
│                        │    │                                   │
│  • 加载出厂标定参数      │    │  • disparity → metric depth       │
│  • 向量化 MEI 反投影    │    │  • depth = (f×T) / disparity      │
│  • radtan 畸变校正      │    │  • f=320px, T=46.3mm             │
│  • cv2.remap 重映射     │    │  • 有效范围: 0.077m ~ 10m         │
└────────────┬───────────┘    └──────────────┬────────────────────┘
             │                               │
             ▼                               ▼
  /fisheye_rect/*/image_raw/compressed   /*/depth/image_raw (可选)
  (JPEG 压缩，默认启用，主输出)
```

---

## 2. 节点说明

### 2.1 seeker_node（C++ 驱动）

| 功能 | 说明 |
|------|------|
| 设备接入 | USB 2207:0000，需要 udev 规则或 sudo |
| 鱼眼图像 | MJPEG 解码后发布为 `/fisheye/*/image_raw`（始终发布，供去畸变节点订阅） |
| 视差图像 | 16UC1 原始数据，`pub_disparity_img=true` 才发布 |
| 视差消息 | `stereo_msgs/DisparityImage`，`pub_disparity=true` 才发布 |
| all/compressed | 设备原始 MJPEG 直传，`pub_fisheye_raw=true` 才发布 |
| IMU | `pub_imu=true` 才发布 |

### 2.2 omni_undistort_node（Python，MEI 模型）

| 功能 | 说明 |
|------|------|
| 相机模型 | MEI（Mei Unified Omnidirectional），参数 xi ≈ 3.2 |
| 标定来源 | 出厂标定，从设备固件读取，存于 `config/seeker_omni_depth/kalibr_cam_chain.yaml` |
| 内参格式 | `[xi, fx, fy, cx, cy]`，畸变 `[k1, k2, p1, p2]` (radtan) |
| 映射计算 | 向量化 NumPy（首帧计算，后续缓存），速度快 |
| 算法流程 | 输出像素 → 逆投影到单位球 → MEI 投影 → radtan 畸变 → 源像素坐标 |
| 边界处理 | `BORDER_REPLICATE` + 无效区域填充原图（避免黑边闪烁） |
| NumPy 兼容 | **不使用 cv_bridge**（NumPy 2.x 不兼容），直接 `np.frombuffer` 转换 |
| 输出格式 | JPEG 压缩（默认质量 80），同时可选输出未压缩原图 |

**相机编号映射：**

| ROS 话题名 | 标定文件 | 方向 |
|-----------|---------|------|
| `/fisheye/left/image_raw` | cam0 | 左 |
| `/fisheye/right/image_raw` | cam1 | 右 |
| `/fisheye/bright/image_raw` | cam2 | 右前 |
| `/fisheye/bleft/image_raw` | cam3 | 左前 |

### 2.3 disparity_to_depth（Python，可选）

| 功能 | 说明 |
|------|------|
| 转换公式 | `depth = (f × T) / disparity`，f=320px，T=46.3mm |
| 无效检测 | 跳过 disparity ≥ 1000（无效标记为 1023.984） |
| 有效范围 | 0.077m ~ 10.0m |
| 运行时控制 | `ros2 param set /disparity_to_depth enabled true/false` |

---

## 3. 话题列表

### 默认启动（最精简）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/fisheye/left/image_raw` | `sensor_msgs/Image` | 原始鱼眼（内部使用） |
| `/fisheye/right/image_raw` | `sensor_msgs/Image` | 原始鱼眼（内部使用） |
| `/fisheye/bright/image_raw` | `sensor_msgs/Image` | 原始鱼眼（内部使用） |
| `/fisheye/bleft/image_raw` | `sensor_msgs/Image` | 原始鱼眼（内部使用） |
| `/fisheye_rect/left/image_raw/compressed` | `sensor_msgs/CompressedImage` | ✅ **主输出** |
| `/fisheye_rect/right/image_raw/compressed` | `sensor_msgs/CompressedImage` | ✅ **主输出** |
| `/fisheye_rect/bright/image_raw/compressed` | `sensor_msgs/CompressedImage` | ✅ **主输出** |
| `/fisheye_rect/bleft/image_raw/compressed` | `sensor_msgs/CompressedImage` | ✅ **主输出** |

### 可选话题（参数开启后）

| 话题 | 开启参数 |
|------|---------|
| `/fisheye_rect/*/image_raw` | `undistort_pub_raw:=true` |
| `/all/compressed` | `pub_fisheye_raw:=true` |
| `/front/disparity` 等 | `pub_disparity:=true` |
| `/front/disparity/image_raw` 等 | `pub_disparity_img:=true` |
| `/front/depth/image_raw` 等 | `use_depth:=true` |
| `/imu_data_raw` | `pub_imu:=true` |

---

## 4. 启动与参数

### 快速启动

```bash
# 编译
cd ~/ros2_ws
colcon build --packages-select seeker --symlink-install
source install/setup.bash

# 默认启动（只输出去畸变压缩图）
ros2 launch seeker 1seeker.launch.py

# 自定义视场（推荐 2.0~2.5）
ros2 launch seeker 1seeker.launch.py undistort_fov_scale:=2.5

# 开启全部功能
ros2 launch seeker 1seeker.launch.py \
  undistort_fov_scale:=2.0 \
  pub_disparity:=true \
  use_depth:=true \
  pub_imu:=true
```

### 完整参数表

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_undistort` | `true` | 启用去畸变节点 |
| `undistort_scale` | `0.5` | 输出分辨率缩放（0.5 = 原图一半） |
| `undistort_fov_scale` | `1.5` | 视场缩放（越大视野越宽，推荐 1.5~2.5） |
| `undistort_jpeg_quality` | `80` | JPEG 压缩质量（1~100） |
| `undistort_pub_raw` | `false` | 是否同时发布未压缩图像 |
| `pub_fisheye_raw` | `false` | 是否发布原始鱼眼图和 all/compressed |
| `pub_disparity` | `false` | 是否发布视差消息 |
| `pub_disparity_img` | `false` | 是否发布视差原始图像 |
| `use_depth` | `false` | 是否启动时就启用深度图转换 |
| `pub_imu` | `false` | 是否发布 IMU 数据 |
| `time_sync` | `true` | 硬件时间同步 |
| `use_rviz` | `false` | 是否启动 RViz2 |

### 运行时动态控制

```bash
# 运行时开启深度图
ros2 param set /disparity_to_depth enabled true

# 运行时关闭深度图
ros2 param set /disparity_to_depth enabled false
```

---

## 5. 标定参数读取

首次使用或更换设备时，从硬件固件读取出厂标定：

```bash
# 确保设备已连接且有访问权限
python3 ~/ros2_ws/src/seeker1/script/1get_kalibr_info.py

# 标定文件自动保存到：
# config/seeker_omni_depth/kalibr_cam_chain.yaml
```

标定参数格式（MEI omni 模型）：
```yaml
cam0:
  camera_model: omni
  intrinsics: [xi, fx, fy, cx, cy]   # xi ≈ 3.2, fx ≈ 1675
  distortion_model: radtan
  distortion_coeffs: [k1, k2, p1, p2]
  resolution: [1088, 1280]
```

---

## 6. 图像查看

```bash
# rqt_image_view（推荐，支持压缩话题）
rqt_image_view
# 下拉菜单选择：/fisheye_rect/left/image_raw/compressed

# 带宽监控
ros2 topic bw /fisheye_rect/bleft/image_raw/compressed
# 预期：~1~2 MB/s（原始未压缩约 18 MB/s）

# 帧率监控
ros2 topic hz /fisheye_rect/left/image_raw/compressed
```

---

## 7. 硬件权限配置（一次性）

```bash
# 添加 udev 规则
sudo tee /etc/udev/rules.d/99-seeker.rules <<EOF
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0000", MODE="0666"
EOF

sudo udevadm control --reload && sudo udevadm trigger
# 重新插拔设备
```

USB 控制器异常恢复：
```bash
sudo modprobe -r xhci_pci && sleep 2 && sudo modprobe xhci_pci
```

---

## 8. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `_ARRAY_API not found` | cv_bridge 与 NumPy 2.x 不兼容 | 已修复：omni_undistort_node 不再使用 cv_bridge |
| `seeker_node` segfault | depth_pubs_ 越界访问 | 已修复：加了空数组保护 |
| 图像闪烁 | 多个 omni_undistort_node 实例 | `pkill -f seeker` 后只启动一次 launch |
| 话题无数据 | 节点未启动 / 设备未连接 | `ros2 node list` 检查节点，`lsusb` 检查设备 |
| 视野太近 | `undistort_fov_scale` 太小 | 增大到 2.0~2.5 |
