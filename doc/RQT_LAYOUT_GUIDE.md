# RQT 图像布局指南

## 在 RQT 中创建 2x4 网格布局

### 第一步：启动系统

```bash
# 终端1: 启动 seeker 节点
cd ~/ros2_ws
source install/setup.bash
ros2 launch seeker 1seeker.launch.py

# 终端2: 启动 rqt
rqt
```

### 第二步：添加 8 个图像视图

1. 点击菜单：`Plugins` → `Visualization` → `Image View`
2. 重复 8 次，每次会创建一个新的面板

### 第三步：选择话题

为每个图像视图选择对应的话题：

**第一排（上排4个）：**
- Image View 1: `/fisheye/left/image_raw`
- Image View 2: `/fisheye/right/image_raw`
- Image View 3: `/fisheye/bright/image_raw`
- Image View 4: `/fisheye/bleft/image_raw`

**第二排（下排4个）：**
- Image View 5: `/front/disparity/image_raw`
- Image View 6: `/right/disparity/image_raw`
- Image View 7: `/back/disparity/image_raw`
- Image View 8: `/left/disparity/image_raw`

### 第四步：排列成 2x4 网格（重要！）

#### 方法：使用拖拽创建网格

1. **准备工作**：
   - 确保所有8个面板都是浮动状态（undocked）
   - 如果某个面板已固定，点击它标题栏的 "×" 旁边的小图标来浮动它

2. **创建第一排（上排）**：
   - 拖动 Panel 1（left）的标题栏到窗口**最上方**，释放
   - 拖动 Panel 2（right）到 Panel 1 的**右边缘**，看到蓝色高亮区域时释放
   - 拖动 Panel 3（bright）到 Panel 2 的**右边缘**，看到蓝色高亮区域时释放
   - 拖动 Panel 4（bleft）到 Panel 3 的**右边缘**，看到蓝色高亮区域时释放

3. **创建第二排（下排）**：
   - 拖动 Panel 5（front）到 Panel 1 的**下边缘**，看到蓝色高亮区域时释放
   - 拖动 Panel 6（right disp）到 Panel 5 的**右边缘**，看到蓝色高亮区域时释放
   - 拖动 Panel 7（back）到 Panel 6 的**右边缘**，看到蓝色高亮区域时释放
   - 拖动 Panel 8（left disp）到 Panel 7 的**右边缘**，看到蓝色高亮区域时释放

4. **调整面板大小**：
   - 拖动面板之间的分隔线来调整每个图像的显示大小
   - 使每个面板大小相等

### 第五步：保存布局

1. 菜单：`Perspectives` → `Export`
2. 保存位置：`~/ros2_ws/src/seeker1/config/my_grid_layout.perspective`
3. 文件名：`my_grid_layout.perspective`

### 下次使用

保存后，下次可以直接加载布局：

```bash
rqt --perspective-file ~/ros2_ws/src/seeker1/config/my_grid_layout.perspective
```

## 布局示意图

```
┌─────────────┬─────────────┬─────────────┬─────────────┐
│   Left      │   Right     │   Bright    │   BLeft     │
│  Fisheye    │  Fisheye    │  Fisheye    │  Fisheye    │
├─────────────┼─────────────┼─────────────┼─────────────┤
│   Front     │   Right     │   Back      │   Left      │
│  Disparity  │  Disparity  │  Disparity  │  Disparity  │
└─────────────┴─────────────┴─────────────┴─────────────┘
```

## 技巧

- **拖拽时注意蓝色高亮区域**：显示面板将被放置的位置
- **边缘拖拽**：拖到其他面板的边缘才会并排显示
- **中心拖拽**：拖到其他面板的中心会创建标签页
- **分隔线**：拖动面板之间的分隔线可以调整大小比例
- **全屏显示**：双击任何图像可以全屏查看，再次双击恢复

## 查看去畸变图像

如果启用了去畸变：

```bash
ros2 launch seeker 1seeker.launch.py use_undistort:=true undistort_fov_scale:=0.35
```

在 rqt 中选择话题：
- `/fisheye_rect/left/image_raw`
- `/fisheye_rect/right/image_raw`
- `/fisheye_rect/bright/image_raw`
- `/fisheye_rect/bleft/image_raw`
