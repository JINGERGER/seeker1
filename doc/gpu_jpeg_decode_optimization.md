# GPU JPEG 解码 + GPU 去畸变优化记录

**日期**：2026-04-27  
**平台**：NVIDIA Jetson AGX Orin (64GB), L4T 36.4.7, CUDA 12.6

---

## 背景

原始代码存在三层问题：
1. 解码阶段：CPU `cv::imdecode` 解码 MJPEG
2. 传输阶段：4 路图像 `cudaMemcpy` 回 CPU + DDS 序列化（CPU 75%）
3. 处理阶段：Python 去畸变节点 `cv2.remap` + `cv2.imencode`（频率下降至 13Hz，延迟 ~1s）

---

## 改动内容

### Phase 1: 异步 GPU 解码 + 减少 memcpy

#### 1.1 替换为 nvJPEG GPU 解码
```cpp
nvjpegDecode(nv_handle_, nv_state_, data, len, NVJPEG_OUTPUT_BGRI, &out, cuda_stream_);
```

#### 1.2 消除中间 cv::Mat，一次 memcpy 搞定
**原来**：GPU → Mat → Image（2 次拷贝）  
**现在**：GPU → Image（1 次拷贝）

#### 1.3 异步解码队列，消除回调线程阻塞
```
设备回调：nvjpegDecode(非阻塞) → cudaEvent 记录 → 入队 → 立即返回
发布线程：cudaEventSynchronize(阻塞) → cudaMemcpy → publish
```

#### 1.4 非 JPEG 帧检测
```cpp
if (len < 2 || data[0] != 0xFF || data[1] != 0xD8) return;
```

**Phase 1 结果**：帧率 16Hz → 17.5Hz，抖动 std dev 0.037s → 0.023s

---

### Phase 2: GPU 去畸变，替换 Python 节点（关键）

#### 2.1 加载标定 + 预计算重映射表（启动时一次）
- yaml-cpp 读取 kalibr 标定参数（xi, fx, fy, cx, cy, distortion）
- CPU 计算 omni 畸变反向映射，上传到 GPU 为 `cv::cuda::GpuMat`

#### 2.2 替换管道
**原来**：
```
GPU raw → cudaMemcpy(CPU) → DDS publish → Python 去畸变 → imencode → compressed
```
**现在**：
```
GPU raw → cv::cuda::remap(GPU) → imencode(CPU) → CompressedImage publish
```

消除：4 路 raw 的 `cudaMemcpy` + DDS 序列化 + Python 节点全部开销。

#### 2.3 新增 ROS 参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pub_rect` | true | 启用 GPU 去畸变输出 |
| `calib_file` | seeker_omni_depth/kalibr_cam_chain.yaml | 标定文件 |
| `undistort_scale` | 0.5 | 输出分辨率缩放 |
| `undistort_fov_scale` | 1.5 | 视场缩放 |
| `jpeg_quality` | 80 | JPEG 质量 |

#### 2.4 CMakeLists 新增依赖
```cmake
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
target_link_libraries(seeker_node opencv_cudawarping opencv_imgcodecs yaml-cpp)
```

#### 2.5 launch 文件
- `use_undistort` 默认改为 false（Python 节点不再需要）
- 新参数透传给 seeker_node

---

## 去畸变数学（MEI omni 模型）
```
iKR = inv(P[:3,:3])
[x,y,w] = iKR * [u,v,1]
r = norm(x,y,w);  Xs=x/r, Ys=y/r, Zs=w/r
denom = Zs + xi
xu = Xs/denom,  yu = Ys/denom
r2 = xu^2+yu^2
xd = (1+k1*r2+k2*r2^2)*xu + 2*p1*xu*yu + p2*(r2+2*xu^2)
yd = (1+k1*r2+k2*r2^2)*yu + p1*(r2+2*yu^2) + 2*p2*xu*yu
map_x = fx*xd + cx,  map_y = fy*yd + cy
```

---

## 性能对比

| 指标 | 原始 | Phase 1 | Phase 2 (最终) |
|------|------|---------|---------------|
| 解码方式 | CPU cv::imdecode | GPU nvJPEG | GPU nvJPEG |
| 去畸变 | Python cv2.remap | Python cv2.remap | GPU cv::cuda::remap |
| CPU 占用 | — | ~75% | **~20%** |
| 平均帧率 | ~16 Hz | ~17.5 Hz | **~20 Hz** |
| 输出延迟 | ~1s | <100ms | <50ms |
| 输出稳定性 | 频率递减 | 稳定 | 稳定 |

---

## 已知问题

**偶发 USB 停顿**：设备每隔一段时间精确丢 1 帧（gap 100-104ms），硬件特性，软件无法修复。可尝试：
```bash
for f in /sys/bus/usb/devices/*/power/control; do echo on | sudo tee $f; done
```
