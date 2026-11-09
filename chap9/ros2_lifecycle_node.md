# ROS 2 生命周期节点：让「启动」变成一件可以检查的事

写 ROS 1 节点时常见这样的场景：`rosrun` 一敲，节点立刻开始发数据，但此时激光
雷达还没连上、参数还没读到、TF 还没起来。你在 RViz 里看到乱跳的点，分不清是
算法错了还是依赖没就绪。

ROS 2 的**托管节点（managed node）** 就是为解决这个问题设计的：节点构造完之后
什么都不做，必须被显式地「配置」和「激活」才开始工作。启停可以反复执行，
每一步的成败都能查。

## 1. 四个状态

![ROS 2 生命周期节点状态机](../img/chap9/ros2_lifecycle_node/lifecycle_states.png)

| 状态 | 含义 | 此时能发布吗 |
|---|---|---|
| `unconfigured` | 刚构造，资源未分配 | 不能 |
| `inactive` | 参数已读、资源已建，但没开始干活 | 不能（publisher 存在但不发） |
| `active` | 正在工作 | 能 |
| `finalized` | 已终止，不可再转移 | 不能 |

四个动作对应四件事：

- `configure`：做**可能失败**的重活——读参数、连传感器、建 publisher。
  失败就停在 `unconfigured`，外部看到的是一个「明确的未就绪」而不是半死不活。
- `activate`：只做**确定能成功**的轻活（启动定时器、打开数据流）。
- `deactivate`：停数据流，但保留资源，随时能再激活。
- `cleanup`：释放资源，退回 `unconfigured`，可重新配置成另一套参数。
- `shutdown`：任意状态都能终止。

关键设计是「重活放 configure、轻活放 activate」。因为 activate 必须能反复成功，
否则启停就不安全；而 configure 允许失败，所以慢的、可能报错的事都放这里。

## 2. 状态机必须严格校验

很多玩具实现遇到非法动作就「静默返回」，结果调用方以为成功了。本模块的做法是
**抛异常并列出当前合法动作**：

```
t=6.5s   configure  状态 inactive 下不能执行 configure（合法动作：activate, cleanup, shutdown）
t=12.0s  activate   状态 unconfigured 下不能执行 activate（合法动作：configure, shutdown）
```

这两条来自演示脚本故意下发的错误指令。实际使用中，这种「响亮地失败」比「悄悄
不生效」省掉大量排查时间——尤其是在 launch 文件里自动编排状态的时候。

合法路径长这样：

```
configure   -> inactive
activate    -> active
deactivate  -> inactive
activate    -> active
deactivate  -> inactive
cleanup     -> unconfigured
configure   -> inactive
activate    -> active
shutdown    -> finalized
```

注意中间「激活 → 停用 → 再激活」跑了两轮，这是生命周期节点相对普通节点最实用的
能力：**换参数重启不需要杀进程**。

## 3. 发布门控：未激活时消息去哪了

演示脚本跑 14 秒，每 0.1 秒尝试发一条心跳，共 141 个 tick：

![发布被接受与被丢弃的累计条数](../img/chap9/ros2_lifecycle_node/lifecycle_publish.png)

| 指标 | 数值 |
|---|---|
| tick 总数 | 141 |
| 发布被接受 | 70 |
| 发布被丢弃 | 71 |
| 激活时间占比 | 49.6% |
| 进入 active 次数 | 2 |
| 合法 / 非法转移 | 7 / 2 |
| 终态 | finalized |

绿线只在 `active` 期间上升，其余时间平坦；红线正好相反。两条曲线加起来恒等于
tick 数——**没有一条消息在「不该发」的时候漏出去**。

状态随时间的台阶：

![状态随时间的变化](../img/chap9/ros2_lifecycle_node/lifecycle_state_timeline.png)

这张图也可以当调试工具用：如果线上数据断了，先看 `ros2 lifecycle get` 的状态，
一眼就能区分「节点没激活」和「节点激活了但数据不对」。

## 4. 代码与运行

```bash
cd src/chap9/ros2_lifecycle_node
python3 tests/test_lifecycle.py   # 26 项单元测试
python3 scripts/main.py           # 打印时间轴并重新生成 3 张图
```

`scripts/lifecycle.py` 把状态机和 ROS 解耦，纯 Python 实现，所以能离线测试：

| 接口 | 说明 |
|---|---|
| `LifecycleStateMachine.trigger(action)` | 执行一次转移，非法则抛 `InvalidTransition` |
| `LifecycleStateMachine.can(action)` | 当前状态下能否执行该动作 |
| `LifecycleStateMachine.actions()` | 列出当前合法动作 |
| `ManagedPublisher.publish(v)` | 非 active 时返回 `False` 并计入 `dropped` |

回调返回 `False` 或抛异常都视为失败，状态保持不变、错误记录在 `machine.errors` 里，
修好之后可以重试——这条也有单测覆盖。

### 在 ROS 2 中跑

```bash
colcon build --packages-select ros2_lifecycle_node
source install/setup.bash
ros2 launch ros2_lifecycle_node ros2_lifecycle_node.launch.py

# 另开终端，按顺序操作
ros2 lifecycle get /lifecycle_heartbeat
ros2 lifecycle set /lifecycle_heartbeat configure
ros2 lifecycle set /lifecycle_heartbeat activate
ros2 topic echo /heartbeat          # 现在才有数据
ros2 lifecycle set /lifecycle_heartbeat deactivate
ros2 lifecycle set /lifecycle_heartbeat cleanup
```

节点启动后 `ros2 topic echo /heartbeat` 是**没有任何输出**的——不是坏了，是还没
激活。想看 configure 失败会怎样：

```bash
ros2 run ros2_lifecycle_node ros_node.py --ros-args -p fail_on_configure:=true
ros2 lifecycle set /lifecycle_heartbeat activate   # 会被拒绝
```

## 5. 什么时候值得用

生命周期节点不是免费的好东西，它让 launch 文件和运维都变复杂了。判断标准：

**值得用**

- 节点启动要做耗时或会失败的初始化（连相机、加载地图、标定）。
- 需要「换配置重启但不重启进程」（比如换地图、换任务）。
- 一个 launch 里有严格依赖：A 配置好了 B 才能激活（配合 `lifecycle_msgs`
  和 `nav2_lifecycle_manager` 自动化）。
- 需要向外部（监控系统）汇报「我到底准备好没有」。

**不值得用**

- 一个纯计算的小工具节点，构造即就绪。
- 教学里演示单个算法——直接写普通节点更省事。

## 6. 局限与延伸

- 本模块只实现四个稳定状态；完整规范里还有 `configuring`、`activating`、
  `deactivating`、`cleaningup`、`errorprocessing` 等过渡态，它们在回调执行期间
  生效。真实 `rclpy.lifecycle` 节点会自动维护这些过渡态。
- 演示脚本里「发布门控」是手写的（`if not active: return False`）。真实项目应使用
  `create_lifecycle_publisher`，由框架保证未激活时不发。
- 没有覆盖 `on_error` 的完整恢复流程（规范里 errorprocessing 成功后进 finalized、
  失败退回 unconfigured）。
- 多节点编排可以用 `nav2_lifecycle_manager` 的 `bond` 机制，节点挂掉能自动降级，
  那是生命周期节点在生产环境里的主要用法。

## 7. 与已有内容的关系

- 与「ROS 2 话题发布订阅基础实验」对照：那篇讲 talker 一上来就发；这篇讲为什么
  真实系统要先把发布「关着」。
- 与「ROS Noetic 环境自检与常见问题排查」互补：ROS 1 里靠 `roswtf` 事后查，
  ROS 2 里靠状态机事前挡。
- 生命周期节点是 Nav2、MoveIt 2 等框架的基础设施，理解了它，看这些框架的
  launch 文件就不至于一头雾水。

> 本文档由 AI 辅助撰写，状态机、发布门控与全部统计数值均由离线单测覆盖
> （`tests/test_lifecycle.py` 26 项全过）。`ros2 lifecycle set` 等 CLI 命令未在
> 安装了 ROS 2 发行版的机器上实测，节点按 `rclpy.lifecycle` 标准接口编写。
