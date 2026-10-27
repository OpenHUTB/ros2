# AirSim无人机Pygame键盘控制模块
## 功能说明
基于pygame实现AirSim无人机手动遥控，规避keyboard库在Linux需要root权限的问题
- W/S：前进、后退
- A/D：左移、右移
- Q/E：上升、下降
- Z/X：机头左转、右转
- ESC：退出程序，自动降落

## 运行环境
Windows端：AirSim仿真环境（AbandonedPark场景）
Ubuntu虚拟机：Python3
```bash
pip install airsim pygame
## 网络配置
虚拟机网卡设置为**桥接模式**，Windows主机和虚拟机处于同一局域网
main.py中默认IP为`192.168.0.11`，根据自己Windows主机IP修改；Windows防火墙放行41451端口

## 运行步骤
1. Windows先启动AirSim仿真器
2. Ubuntu进入模块目录
```bash
cd src/airsim_drone_keyboard_control
python main.py
## 演示效果图
![运行截图](demo.png)

> 声明：本文档和代码部分由大模型辅助生成，本人对提交内容全部负责
