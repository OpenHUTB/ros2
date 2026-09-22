<!-- ⚠️ 请勿删除此模板。提交前请逐项核对，缺失可能导致 PR 未通过审核。 -->

## 标题（Pull Request 标题请具体说明改了什么，如）
`carla 0.9.16 无人车：作业二感知/控制改为神经网络(纯numpy MLP)+离线可训练与在线运行`

## 修改概述（一句话）

## 详细修改
1. 《本次改了哪个模块、为什么》

## 运行方式（每个模块必须支持 main.* 直接运行整个模块）
```bash
# 需先启动 CARLA 0.9.16 服务端（在线 run 需要；train 模式无需 CARLA）
./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low

cd carla_assignment
source .venv/bin/activate

# 离线训练两个神经网络（无需 CARLA，仅 numpy）
python 02_perception/main.py --mode train
python 03_navigation/main.py --mode train

# 根入口（各作业）
python main.py --task control                       # 作业一
python main.py --task perception  --mode train     # 作业二
python main.py --task navigation   --mode train     # 作业三
python main.py --task end_to_end  --mode test      # 作业四

# 或逐模块 + launch（ROS1 Noetic / ROS2 Humble）
python 01_control/main.py
ros2 launch carla_assignment 02_perception_launch.py mode:=run   # ROS2 Humble
roslaunch carla_assignment 01_control.launch                     # ROS1 Noetic
```

## 运行效果（必须在提交信息中提供运行效果图）
<!-- 贴动图（<10MB）或截图： -->
![运行效果](docs/assets/xx.gif)

## 经过了什么测试
1. 操作系统：Ubuntu 20.04 / 22.04 或 Windows 10+ · CARLA 0.9.16
2. 基础校验：两个 NN `--mode train` 离线跑通，launch 语法检查
3. **是否已由另一位同学在另一台机器测试并同意运行**：是 / 否

## 环境
```bash
pip install -r requirements.txt
# carla 模块从 CARLA 发行包 PythonAPI 安装
pip install /path/to/Carla/PythonAPI/carla/dist/*.whl
```

## 大模型使用声明
无 / 使用了某大模型辅助生成代码与文档
