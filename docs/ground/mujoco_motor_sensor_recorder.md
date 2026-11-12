# 电机传感器 CSV 记录器

记录器将通用电机状态样本转换为可复现的温度和电流观测值，直接读取 CSV，不依赖仿真器或其他同级包。

## 验证

在 Ubuntu 22.04、ROS 2 Humble、Python 3.10.12 和 pytest 6.2.5 容器中，colcon 构建及 5 项测试通过。使用相同随机种子的重复运行会生成字节级一致的输出；零转矩下的电流保持在空载电流附近；输入和输出路径相同时会拒绝运行并保留原文件。

构建后执行 `colcon test --merge-install --packages-select mujoco_motor_sensor_recorder --python-testing pytest --return-code-on-test-failure` 和 `colcon test-result --verbose`，结果为 5 tests、0 errors、0 failures。

## 运行结果

```text
time_s,speed_mps,left_temperature_c,right_temperature_c,left_current_a,right_current_a,ambient_c
0.000000,0.200000,24.969294,25.161372,1.240956,2.237397,24.000000
0.100000,0.240000,24.988398,25.174404,1.405588,2.044744,24.000000
0.200000,0.290000,25.324425,25.429868,1.488013,1.924080,24.000000
0.300000,0.310000,25.200072,25.702630,1.659144,1.825508,24.000000
0.400000,0.340000,25.397036,25.590733,1.769971,1.675717,24.000000
```

输出温度和电流字段按指定随机种子加入高斯测量噪声。以上五条记录由随包提供的 CSV 示例运行生成，所有数值字段保留六位小数，以消除常见的平台浮点末位差异。

本地使用时，在包目录执行 `python -m pip install .`，再运行 `motor_sensor_record --input config/motor_state.csv --output output/motor_sensors.csv --seed 7`。CSV 输入路径相对于当前工作目录。
