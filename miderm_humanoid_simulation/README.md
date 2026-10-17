MuJoCo人形机器人仿真
项目简介：
基于MuJoCo搭建简易人形机器人，完成动态仿真，本次作业不使用Gazebo。
 
文件结构：
- humanoid_sim：仿真代码，sim.py为主程序
- docs：实验报告、运行日志、仿真视频
- pic：截图素材
 
运行方式：
1. 安装依赖：
```bash
pip install mujoco numpy imageio[ffmpeg]
2.执行仿真代码：cd humanoid_sim
python3 sim.py

环境
Ubuntu22.04 + MuJoCo