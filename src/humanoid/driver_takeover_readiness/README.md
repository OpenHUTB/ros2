# driver_takeover_readiness

驾驶人接管准备度评分模块，综合注视、手在方向盘、脚在踏板和请求后的定向时间。
输出可解释的 `score`、`ready`、`level` 和原因列表。

## 运行

```bash
cd src/humanoid/driver_takeover_readiness
python main.py --gaze-on-road --hands-on-wheel --feet-on-pedals --alert-age 2.0
```

## 测试

```bash
python -m unittest discover -s test -p "test_*.py" -v
```

ROS 2 话题：`~/observation` 接收 `[gaze, hands, feet, alert_age, gaze_off]`，
`~/readiness` 发布 JSON `String`。
