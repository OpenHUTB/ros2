# 自动驾驶演示

## 随机路线
```shell
roslaunch carla_ad_demo carla_ad_demo.launch host:=172.21.108.47 timeout:=60000 town:='Carla/Maps/Town10HD_Opt' spawn_point:=-25,-134,0.5,0,0,-90
```


## 场景执行

```shell
roslaunch carla_ad_demo carla_ad_demo_with_scenario.launch host:=172.21.108.47 timeout:=60000 town:='Carla/Maps/Town10HD_Opt'
```

报错：
```log
RLException: Invalid <arg> tag: environment variable 'SCENARIO_RUNNER_PATH' is not set. 

Arg xml is <arg name="scenario_runner_path" default="$(env SCENARIO_RUNNER_PATH)"/>
The traceback for the exception was written to the log file
```

原因：必须指定`SCENARIO_RUNNER_PATH`，指向 scenario runner 的路径。

## 参考

* [Carla 自动驾驶演示](https://openhutb.github.io/doc/carla_ad_demo/)