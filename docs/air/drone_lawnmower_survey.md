# AirSim 无人机弓字形区域勘测

`drone_lawnmower_survey` 根据方形或矩形区域尺寸生成往返覆盖航线，使 AirSim
多旋翼在固定高度完成自动勘测。源代码位于
`src/air/drone_lawnmower_survey/`。

实现思路参考 Microsoft AirSim Wiki 的
[Implementing a Drone Survey script](https://github.com/microsoft/AirSim/wiki/Implementing-a-Drone-Survey-script)。
与原教程片段相比，本模块增加了参数边界校验、安全预览、相对起飞点定位、自动降落和
不依赖 AirSim 的航线单元测试。

## 航线生成

输入参数包括区域长度、区域宽度、航线间距、飞行高度和速度。生成器把勘测区域放在
起飞点周围，并沿 NED x 轴交替飞行；每完成一条航线，就沿 y 轴移动到下一条航线。
当区域宽度不能被指定间距整除时，程序会减小实际间距，以确保两侧边界都在覆盖范围内。

AirSim 使用 NED 坐标系，z 轴向下为正，因此用户输入的正高度会转换成负 z 坐标。

## 安全预览

默认命令只输出航点、路径长度和预计飞行时间，不连接 AirSim：

```bash
python src/air/drone_lawnmower_survey/main.py \
  --length 60 --width 40 --lane-spacing 8 --altitude 20 --speed 5
```

默认安全预览运行效果：

![默认安全预览运行效果](../img/air/drone_lawnmower_survey/preview.png)

所有尺寸、高度和速度必须是有限正数。`nan`、`inf`、零和负数会在连接仿真器前被拒绝。

## 执行勘测

安装 AirSim Python API 并启动多旋翼场景：

```bash
pip install airsim
python src/air/drone_lawnmower_survey/main.py --execute \
  --length 60 --width 40 --lane-spacing 8 --altitude 20 --speed 5
```

脚本依次启用 API 控制、解锁、起飞、爬升、执行路径、悬停和降落。无论正常结束还是
任务抛出异常，都会尝试降落、锁定电机并释放 API 控制权。

> 当前模块不包含障碍物感知与避障。首次执行应选择空旷场景，先检查预览航点是否落在
> 安全区域，再添加 `--execute`。

## 测试

```bash
python -m unittest discover -s src/air/drone_lawnmower_survey/tests -v
```

测试验证矩形边界覆盖、航向交替、实际间距上限、坐标平移、路径长度和非法参数拒绝，
无需启动 AirSim。
