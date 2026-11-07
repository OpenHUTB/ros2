# two_link_arm_ik

二连杆机械臂正向与逆向运动学 2D 仿真。基座固定，给定末端目标点，用解析逆解算出两个关节角，机械臂逐帧插值过去，末端留下运动轨迹。

## 运行

```bash
cd src/ground/two_link_arm_ik
python main.py        # Windows 用 main.bat，Linux/macOS 用 main.sh
```

依赖：`pip install -r requirements.txt`（仅 pygame）。

## 测试

```bash
python -m pytest test/test_arm_kinematics.py
```

覆盖：全伸展姿态的末端位置、直角姿态的末端位置、正向解的逆解回代、工作空间外目标返回空解、边界可达性判定、关节限位裁剪，以及依次抵达全部目标点。
