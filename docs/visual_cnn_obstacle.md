# 基于简单CNN的视觉避障（ROS）
## 项目简介
本项目在ROS环境下实现一套视觉避障程序。采用自制图像发布节点生成模拟图像，通过简单CNN对图像进行推理，判断前方是否存在障碍物，输出避障决策。本项目暂未接入第三方仿真软件，依靠程序生成虚拟图像作为视觉输入，验证节点通信、图像订阅、CNN前向推理、结果发布整套流程。

## 环境
- Ubuntu 20.04
- ROS Noetic
- Python3
- OpenCV‑Python
- PyTorch

## 文件结构
vision_cnn_avoid/
├── scripts/
│   ├── img_pub.py        # 模拟图像发布节点，发布 /image_raw
│   └── cnn_avoid.py      # CNN避障节点，订阅图像，推理并输出结果
├── CMakeLists.txt
└── package.xml

## ROS话题
- 订阅：`/image_raw`（sensor_msgs/Image，模拟摄像头图像）
- 发布：可根据需求新增避障指令话题

## 核心代码
### 1.img_pub.py
```python
#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

def cv2_to_msg(cv_img):
    msg = Image()
    msg.height = cv_img.shape[0]
    msg.width = cv_img.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = cv_img.shape[1] * 3
    msg.data = cv_img.tobytes()
    return msg

if __name__ == "__main__":
    rospy.init_node("img_pub_node")
    pub = rospy.Publisher("/image_raw", Image, queue_size=1)
    rate = rospy.Rate(5)
    h, w = 480, 640
    rospy.loginfo("Image publisher start.")
    while not rospy.is_shutdown():
        frame = np.ones((h, w, 3), dtype=np.uint8)*255
        cv2.rectangle(frame, (270, 180), (370, 300), (0, 0, 0), -1)
        img_msg = cv2_to_msg(frame)
        pub.publish(img_msg)
        rate.sleep()
```     
### 2.cnn_avoid.py
```python
#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import torch
import torch.nn as nn
from sensor_msgs.msg import Image

class SimpleCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 16, 3, padding=1)
        self.conv2 = nn.Conv2d(16, 32, 3, padding=1)
        self.pool = nn.MaxPool2d(2,2)
        self.fc1 = nn.Linear(32*120*160, 64)
        self.fc2 = nn.Linear(64, 2)

    def forward(self, x):
        x = self.pool(torch.relu(self.conv1(x)))
        x = self.pool(torch.relu(self.conv2(x)))
        x = x.view(x.size(0), -1)
        x = torch.relu(self.fc1(x))
        x = self.fc2(x)
        return x

model = SimpleCNN()
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)
model.eval()

def callback(msg):
    h = msg.height
    w = msg.width
    arr = np.frombuffer(msg.data, np.uint8)
    img = arr.reshape((h,w,3))
    img = cv2.resize(img,(320,240))
    img_rgb = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    tensor = torch.from_numpy(img_rgb).permute(2,0,1).float().unsqueeze(0)/255.0
    tensor = tensor.to(device)
    with torch.no_grad():
        out = model(tensor)
        pred = torch.argmax(out, dim=1).item()
    if pred == 1:
        rospy.loginfo("Detect obstacle! Need to avoid.")
    else:
        rospy.loginfo("No obstacle, go straight.")

if __name__ == "__main__":
    rospy.init_node("cnn_obstacle_avoidance_node")
    rospy.loginfo("CNN obstacle‑avoidance node started")
    sub = rospy.Subscriber("/image_raw", Image, callback, queue_size=1)
    rospy.spin()
``` 
    
### 3.CMakeLists.txt
```python
 cmake_minimum_required(VERSION 3.0.2)
project(vision_cnn_avoid)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/img_pub.py
  scripts/cnn_avoid.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
``` 

### 4.package.xml
```python
<?xml version="1.0"?>
<package format="2">
  <name>vision_cnn_avoid</name>
  <version>0.0.0</version>
  <description>The vision_cnn_avoid package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
</package>
``` 

## 编译与运行命令
## 1.进入工作空间编译
cd ~/vision_ws
catkin_make
## 2. 刷新环境变量
source devel/setup.bash
## 3.启动ROS核心
roscore
### [Commit提交记录](https://github.com/tangwang-0614/ros2/commit/6f6cddd5b75678d6ae34d476dfe1024f06872264)
