#!/usr/bin/env python3
"""
Simple script to view all seeker images in one rqt window
Opens rqt and you can manually add Image View plugins for each topic
"""

import subprocess
import sys
import time

print("=" * 60)
print("Seeker Image Viewer")
print("=" * 60)
print("\n启动rqt后，请按以下步骤操作：")
print("\n1. 点击菜单: Plugins -> Visualization -> Image View (添加8次)")
print("2. 为每个图像视图选择对应的话题:")
print("   - /fisheye/left/image_raw")
print("   - /fisheye/right/image_raw")
print("   - /fisheye/bright/image_raw")
print("   - /fisheye/bleft/image_raw")
print("   - /front/disparity/image_raw")
print("   - /right/disparity/image_raw")
print("   - /back/disparity/image_raw")
print("   - /left/disparity/image_raw")
print("\n3. 拖动面板排列成 2行x4列 的网格布局")
print("4. 保存布局: Perspectives -> Export")
print("=" * 60)
print("\n正在启动 rqt...\n")

time.sleep(3)

# Launch rqt
subprocess.run(['rqt'])
