# Seeker 真机自检脚本

本文档说明如何使用 `script/seeker_hw_self_check.sh` 对 Seeker ROS2 管线做一键真机验收。

## 1. 脚本功能

脚本会自动执行以下检查：

- 检查基础命令是否存在：`ros2`、`timeout`、`lsusb`
- 检查工作空间是否已编译（`install/setup.bash`）
- 检查 USB 设备是否识别（`2207:0000`）
- 启动 `ros2 launch seeker 1seeker.launch.py`
- 检查关键节点是否运行：
  - `/seeker_node`
  - `/omni_undistort_node`
  - `/disparity_to_depth`
- 检查关键话题是否存在：
  - `/fisheye_rect/left/image_raw/compressed`
  - `/fisheye_rect/right/image_raw/compressed`
  - `/fisheye_rect/bright/image_raw/compressed`
  - `/fisheye_rect/bleft/image_raw/compressed`
- 抽样话题频率和带宽（左前去畸变压缩图）

最后给出 `PASS/WARN/FAIL` 汇总，并自动结束 launch 进程。

## 2. 使用方法

在工程目录执行：

```bash
cd ~/ros2_ws/src/seeker1
chmod +x script/seeker_hw_self_check.sh
./script/seeker_hw_self_check.sh
```

## 3. 常见用法

### 指定工作空间

```bash
./script/seeker_hw_self_check.sh -w ~/ros2_ws
```

### 调整采样时间（秒）

```bash
./script/seeker_hw_self_check.sh -t 10
```

### 透传 launch 参数

```bash
./script/seeker_hw_self_check.sh -- undistort_fov_scale:=2.0 use_depth:=true
```

### 指定 launch 文件

```bash
./script/seeker_hw_self_check.sh -l 1seeker.launch.py
```

## 4. 结果判读

- `FAIL > 0`：本次验收失败，需要先处理报错项。
- `WARN > 0` 且 `FAIL = 0`：可运行，但建议根据警告优化（例如 USB 识别状态、带宽抽样不足）。
- `FAIL = 0`：真机自检通过。

## 5. 日志位置

- Launch 日志：`/tmp/seeker_hw_self_check.launch.log`

如启动异常，可先查看该日志定位原因。
