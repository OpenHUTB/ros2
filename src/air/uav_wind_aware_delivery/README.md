# uav_wind_aware_delivery

四旋翼风扰下物资精准空投：MuJoCo 吊挂/释放、ROS Noetic 全链、弹道与 NumPy MLP 落点预测、释放点补偿。

完整教程（环境、接口、指标、报错、限制）：

**`docs/air/uav_wind_aware_delivery.md`**

## 快速启动

```bash
# 放入 catkin 工作区
mkdir -p ~/air_ws/src
cp -r <repo>/src/air/uav_wind_aware_delivery ~/air_ws/src/

# 编译
cd ~/air_ws
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash

# 完整演示（OpenCV Viewer）
roslaunch uav_wind_aware_delivery demo.launch wind_x:=3.0 wind_y:=1.0

# 无窗口
roslaunch uav_wind_aware_delivery headless.launch

# 测试/训练/评估
export UAV_VENV_PYTHON=/path/to/python3.11
$UAV_VENV_PYTHON tests/run_all.py
$UAV_VENV_PYTHON tools/train_predictor.py
$UAV_VENV_PYTHON tools/evaluate.py
```

## 结果

- 落点预测验证 RMSE **0.0143 m**（零偏移基线 0.8661 m，提升 98.35%）
- 10 组随机风：NN 补偿 **10/10 成功，平均 0.1604 m**；不补偿 **0/10，1.0455 m**
- 平均误差降低 **84.7%**
- 15/15 单元测试通过

## 说明

- 支持 ROS Noetic + MuJoCo 3.12 + Python 3.11 venv；不修改系统 Python/ROS。
- 控制器为简化几何控制（世界系合力/力矩），重点在空投任务与落点预测，不是飞控级混控模型。
- 视频：`media/ubuntu_air_demo.mp4`；GIF：`docs/img/air/uav_wind_aware_delivery.gif`。

MIT License.
