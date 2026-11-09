# 任务说明：行人隐私脱敏视觉节点
## 任务目标
基于YOLOv8分割模型实现一个ROS2视觉节点，对图像中的行人目标进行检测，并对行人区域做高斯模糊处理，实现隐私保护。原始图像与脱敏后的图像分别以压缩图像话题发布，可在 rqt_image_view 中查看效果。

## 功能要求
1. 节点内加载一张静态测试图片；
2. 使用 YOLOv8n-seg 进行目标检测，仅筛选行人类别（类别id=0）；
3. 对检测到的行人包围盒区域进行高斯模糊；
4. 发布两个 CompressedImage 话题：
   - `/image_raw/compressed`：原始输入图像
   - `/image_masked/compressed`：行人模糊后的脱敏图像
5. 通过 launch 文件一键启动节点。

## 功能包结构
privacy_mask/

├── privacy_mask/

│   ├── __init__.py

│   └── privacy_mask_node.py
├── launch/

│   └── privacy_mask.launch.py

├── resource/

│   └── privacy_mask

├── package.xml

├── setup.cfg

├── setup.py

└── README.md

## 依赖环境
- ROS2 Humble
- OpenCV-Python
- ultralytics（YOLOv8）
- rclpy、sensor_msgs

## 运行步骤
```bash
colcon build --packages-select privacy_mask
source install/setup.bash
ros2 launch privacy_mask privacy_mask.launch.py
