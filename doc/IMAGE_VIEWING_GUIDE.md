# Seeker 图像查看指南

## 方法1: 使用 rqt 查看所有图像 (推荐)

### 步骤:

1. **启动 seeker 节点:**
```bash
ros2 launch seeker 1seeker.launch.py
```

2. **在新终端启动 rqt:**
```bash
rqt
```

3. **添加图像视图 (重复8次):**
   - 点击菜单: `Plugins` -> `Visualization` -> `Image View`
   - 每次会创建一个新的图像视图面板

4. **为每个面板选择话题:**
   
   上排4个:
   - Panel 1: `/fisheye/left/image_raw`
   - Panel 2: `/fisheye/right/image_raw`  
   - Panel 3: `/fisheye/bright/image_raw`
   - Panel 4: `/fisheye/bleft/image_raw`
   
   下排4个:
   - Panel 5: `/front/disparity/image_raw`
   - Panel 6: `/right/disparity/image_raw`
   - Panel 7: `/back/disparity/image_raw`
   - Panel 8: `/left/disparity/image_raw`

5. **排列布局:**
   - 拖动每个面板的标题栏
   - 将它们排列成 2行 x 4列 的网格
   - 上面4个摄像头图像，下面4个深度图像

6. **保存布局 (可选):**
   - 菜单: `Perspectives` -> `Export`
   - 保存到: `~/ros2_ws/src/seeker1/config/my_seeker_layout.perspective`
   - 下次启动: `rqt --perspective-file ~/ros2_ws/src/seeker1/config/my_seeker_layout.perspective`

## 方法2: 使用去畸变图像

如果想查看去畸变后的图像:

```bash
# 启动seeker节点和去畸变节点
ros2 launch seeker 1seeker.launch.py use_undistort:=true undistort_fov_scale:=0.4

# 在rqt中选择这些话题:
# - /fisheye_rect/left/image_raw
# - /fisheye_rect/right/image_raw
# - /fisheye_rect/bright/image_raw
# - /fisheye_rect/bleft/image_raw
```

## 参数调整

### 去畸变参数:
- `undistort_fov_scale`: 视场缩放 (0.3-0.5, 越小越放大，避免圆形边缘)
  ```bash
  ros2 launch seeker 1seeker.launch.py use_undistort:=true undistort_fov_scale:=0.3
  ```

- `undistort_scale`: 输出图像缩放 (0.5 = 一半分辨率)
  ```bash
  ros2 launch seeker 1seeker.launch.py use_undistort:=true undistort_scale:=0.5
  ```

## 快速启动命令

### 仅原始图像:
```bash
ros2 launch seeker 1seeker.launch.py && rqt
```

### 带去畸变:
```bash
ros2 launch seeker 1seeker.launch.py use_undistort:=true undistort_fov_scale:=0.35 && rqt
```

## 提示

- 第一次设置好布局后，一定要导出perspective文件保存
- rqt允许你自由拖动和调整每个面板的大小
- 可以关闭不需要的面板
- 使用 `Ctrl + Mouse Wheel` 可以在图像上缩放
