# Carla 的 ROS 桥学习笔记（含 C++ 实现注释）

我在完成小海龟的基础实验后，按照作业附加题的要求学习了 Carla 模拟器接入 ROS 的方式。这篇笔记的思路是先整理两种 ROS 桥的基本情况，再对照源码仓库，给 C++ 实现的关键代码加上中文注释。本文是 [小海龟仿真实验报告](./turtle_sim_experiment.md) 的附录。

## 1. ROS 桥是干什么的

CARLA 是基于 Unreal Engine 的开源自动驾驶模拟器，它内部的世界（车辆、传感器、物理仿真）对 ROS 没有认识；反过来，ROS 生态里的感知、规划、控制算法也读不懂 CARLA 的私有数据。ROS 桥（ROS Bridge）就是两者之间的翻译官，提供双向通信：

- 上行：把 CARLA 里的车辆状态、相机/LiDAR/GNSS/IMU 等传感器数据，转成标准 ROS 话题（sensor_msgs 等）发出去，供 RViz、导航栈这些节点使用；
- 下行：订阅 ROS 侧的控制指令话题，翻译之后调用 CARLA 的接口去控制车辆（转向、油门、刹车）。

## 2. Python 版 ROS 桥（carla-simulator/ros-bridge）

这是历史最长、资料最多的官方方案，基于 CARLA 的 Python API 实现，ROS 1 和 ROS 2 都支持，包含几个功能包：

| 功能包 | 职责 |
| :--- | :--- |
| carla_ros_bridge | 核心桥：actor 管理、传感器数据从 CARLA 流转成 ROS 话题、TF 坐标维护 |
| carla_ackermann_control | 订阅 ackermann_cmd，换算成 vehicle_control 下发给 CARLA 车辆 |
| carla_ad_agent | 一个简单的自动驾驶代理示例（跟随前车） |
| carla_manual_control | 键盘手动控制的示例节点 |

话题命名遵循 /carla/&lt;role_name&gt;/... 的约定，比如相机图像发布在 /carla/&lt;role_name&gt;/camera/rgb/image_raw，控制指令通过 /carla/&lt;role_name&gt;/vehicle_control_cmd（或 ackermann 版）下发。

## 3. C++ 实现：LibCarla/source/carla/ros2

从 CARLA 的 UE5 版本（0.10 起）开始，模拟器内置了原生 C++ 的 ROS2 桥，源码在 LibCarla/source/carla/ros2，基于 Fast DDS 实现，不再绕道 Python API——传感器数据在 C++ 内部直接序列化发布，避开了 Python 的 GIL 和数据拷贝开销，对相机、LiDAR 这种高频大数据流收益明显。

实际查看 ue5-dev 分支的源码，目录结构是这样的：

```text
LibCarla/source/carla/ros2/
├── ROS2.cpp / ROS2.h        # 核心：ROS2 单例类（注册、话题命名、数据分发）
├── ROS2CallbackData.h       # 回调数据结构
├── FastDDSAliases.h         # Fast DDS 类型别名封装
├── publishers/              # 38 个文件：各类传感器的发布器
├── subscribers/             # 订阅器（车辆控制指令等下行数据）
├── listeners/               # DDS 底层监听器
└── types/                   # 消息类型定义
```

publishers 目录按传感器类型组织，数了一下主要有这几类：

| 分类 | 发布器 |
| :--- | :--- |
| 相机类 | CarlaRGBCameraPublisher、CarlaDepthCameraPublisher、CarlaSSCameraPublisher（语义分割）、CarlaISCameraPublisher（实例分割）、CarlaDVSCameraPublisher（事件相机）、CarlaOpticalFlowCameraPublisher、CarlaNormalsCameraPublisher |
| 测距类 | CarlaLidarPublisher、CarlaSemanticLidarPublisher、CarlaRadarPublisher |
| 定位惯导 | CarlaGNSSPublisher、CarlaIMUPublisher、CarlaTransformPublisher（TF） |
| 事件类 | CarlaCollisionPublisher、CarlaPointCloudPublisher |
| 基础类 | BasePublisher.h、BasicPublisher、CarlaClockPublisher、PublisherImpl.h |

## 4. 关键类注释

### 4.1 早期文档和当前版本的类名对照

OpenHUTB 中文文档《[RosBridge 以 C++ 实现](https://carla-openhutb.readthedocs.io/zh-cn/latest/ros/bridge_cpp/)》里介绍的五个关键类属于 0.10 早期版本，现在 ue5-dev 分支已经重构过了。对照如下：

| 早期类（文档版） | 当前版本对应 | 职责 |
| :--- | :--- | :--- |
| RosUtils | 工具函数（如 BuildBaseTopicName、LookupFrameId） | 话题命名、frame_id 解析这类公共逻辑 |
| RosSink | ROS2 单例的 ProcessDataFrom* 系列入口 | 接收 CARLA 数据并分发给对应的发布器 |
| RosPublisher | BasePublisher / Carla*Publisher 系列 | 按传感器类型发布 ROS2 话题 |
| RosSubscriber | BasicSubscriber / subscribers 目录 | 订阅下行控制话题 |
| RosAction | 重构后并入了回调机制 | ROS action 语义的封装 |

### 4.2 ROS2.h 单例类逐段注释

下面的注释是我对照 LibCarla/source/carla/ros2/ROS2.h（ue5-dev 分支）的源码整理的，为了方便阅读做了精简：

```cpp
namespace carla {
namespace ros2 {

/// ROS2 桥的总入口：一个惰性加载的单例。
/// 整个模拟器进程内只有一个 ROS2 实例，所有传感器/车辆都注册到它身上。
class ROS2 {
public:
  ROS2(const ROS2&) = delete;              // 禁止拷贝，保证全局唯一
  static std::shared_ptr<ROS2> GetInstance();  // 惰性单例：第一次调用才构造

  /// 总开关和生命周期
  void Enable(bool enable);                // 开关 ROS2 桥
  void Shutdown();                         // 释放所有 DDS 参与者
  bool IsEnabled() const;
  void SetFrame(uint64_t frame);           // 当前仿真帧号 → ROS 消息的 header
  void SetTimestamp(...);                  // 仿真时间戳（配合 /clock 话题）

  /// ============ 注册接口 ============
  /// 注册传感器：按 stream_id 登记 ros_name 和 frame_id
  /// （取代了早期版本的 AddActorRosName / GetActorRosName 接口）
  void RegisterSensor(...);
  void UnregisterSensor(...);
  /// 注册车辆；enable_ackermann_control 决定挂哪个控制订阅器：
  /// true → Ackermann 控制订阅器，false → 直接 vehicle_control 订阅器（二选一）
  void RegisterVehicle(..., bool enable_ackermann_control);
  void UnregisterVehicle(...);
  /// 登记 actor 的父 actor：话题名会带上父级前缀，形成层级命名
  void AddActorParentRosName(...);

  /// ============ 数据上行（CARLA → ROS2）============
  /// 这些是"数据水龙头"：模拟器每产生一帧传感器数据就调用对应入口，
  /// 内部拿到（或创建）该传感器的发布器后发布。
  void ProcessDataFromCamera(...);              // 相机图像 → sensor_msgs/Image
  void ProcessDataFromGNSS(...);                // GNSS → sensor_msgs/NavSatFix
  void ProcessDataFromIMU(...);                 // IMU → sensor_msgs/Imu
  void ProcessDataFromLidar(...);               // LiDAR → sensor_msgs/PointCloud2
  void ProcessDataFromSemanticLidar(...);
  void ProcessDataFromRadar(...);               // → carla_msgs/CarlaRadarMeasurement
  void ProcessDataFromDVSEvent(...);            // 事件相机
  void ProcessDataFromObstacleDetection(...);   // 物体检测事件
  void ProcessDataFromCollisionSensor(...);     // 碰撞事件

private:
  /// 一次注册 = 话题名 + TF 名 + 是否发布 TF
  struct ActorRegistration {
    std::string ros_name;
    std::string frame_id;
    bool publish_tf{true};
  };

  /// 话题命名规则：rt/carla/[父级链/]ros_name
  /// 比如挂在高空无人机下的相机，话题会带上级的命名空间
  std::string BuildBaseTopicName(...);

  /// 惰性创建发布器：同一传感器第一次来数据时才真正构造 DDS 发布器
  std::shared_ptr<CameraT> GetOrCreateCameraSensor<CameraT>(...);

  /// 成员：注册表、父级关系表、时钟发布器、各类发布器缓存
  std::unordered_map<...> _registrations;
  std::unordered_map<...> _actor_parents;
  std::shared_ptr<CarlaClockPublisher> _clock_publisher;  // 发布仿真时钟 /clock
  std::unordered_map<...> _publishers;
  std::unordered_map<...> _camera_publishers;
  std::unordered_map<...> _transforms;                    // TF 发布器
  /// 以下只在编译期定义了 WITH_ROS2_DEMO 时参与构建（演示用）
  std::shared_ptr<BasicSubscriber> _basic_subscriber;
  std::shared_ptr<BasicPublisher> _basic_publisher;
};
}  // namespace ros2
}  // namespace carla
```

读完这个头文件，我发现有几个设计上的点值得记下来。比如整个桥的注册（RegisterSensor/RegisterVehicle）、命名（BuildBaseTopicName）、分发（ProcessDataFrom*）全部收口到一个 ROS2 单例里，多传感器不会各管各的；发布器是"第一次来数据才创建"的（GetOrCreate 模式），没被用到的传感器不占 DDS 资源；话题名通过父级链拼成 rt/carla/[parent/]ros_name，多车、多传感器的命名空间天然有序；一辆车的控制订阅在 Ackermann 和直接控制之间二选一，避免两路指令打架。

### 4.3 数据流

```text
【上行：传感器数据】
CARLA 模拟器（每帧 tick）
   │  传感器原始数据（相机图像、LiDAR 点云、GNSS/IMU、碰撞事件……）
   ▼
ROS2 单例 ProcessDataFromCamera / GNSS / IMU / Lidar / ...
   │  GetOrCreate*Publisher（惰性创建对应发布器）
   ▼
publishers/* 各类发布器
   │  Fast DDS 序列化后发布
   ▼
ROS2 话题 /carla/<role>/...  ──→  RViz / 感知 / 导航栈

【下行：控制指令】
ROS2 话题 /carla/<role>/ackermann_control_cmd
   ▼
subscribers/* 控制订阅器
   │  换算成 VehicleControl
   ▼
CARLA 车辆执行（vehicle.apply_control）
```

## 5. 和 Python 版的对比

| 维度 | Python 版 ros-bridge | C++ 版（LibCarla/ros2） |
| :--- | :--- | :--- |
| 集成方式 | 独立节点，经 CARLA Python API 拉取数据 | 编译进模拟器核心，原生集成 |
| 性能 | 高频大带宽传感器下受 GIL/拷贝限制 | C++ 直连 Fast DDS，开销小 |
| ROS 支持 | ROS1 + ROS2 | 仅 ROS2 |
| 功能包生态 | ackermann 控制、AD agent、手动控制等成熟示例 | 侧重传感器/控制数据通路 |
| 适用场景 | 快速原型、教学演示 | 对实时性要求高的正式开发 |

## 6. 小结

这次学习最大的收获是搞清楚了 ROS 桥的本质：它做的其实是协议翻译加命名空间管理——把 CARLA 的 actor 世界映射成 ROS 的节点/话题世界，话题命名规则（rt/carla/[parent/]ros_name）是两边对得上的关键。C++ 版把"每帧经 Python API 拉取"变成了"模拟器帧内直发"（tick 时就地调 ProcessDataFrom*），这就是它性能优势的来源。另外从早期五个类（RosUtils/RosSink/RosAction/RosSubscriber/RosPublisher）到现在的单例 ROS2 加按传感器划分的 publishers/subscribers，能看出"中心化注册 + 惰性创建"这种重构思路，对写大型机器人系统挺有参考价值。

## 7. 参考资料

1. carla-simulator/ros-bridge（Python 版官方仓库，https://github.com/carla-simulator/ros-bridge）
2. CARLA 文档：RosBridge 以 C++ 实现（OpenHUTB 中文镜像，https://carla-openhutb.readthedocs.io/zh-cn/latest/ros/bridge_cpp/）
3. carla-simulator/carla 仓库 LibCarla/source/carla/ros2 源码（ue5-dev 分支，https://github.com/carla-simulator/carla/tree/ue5-dev/LibCarla/source/carla/ros2）

## 8. 声明

本报告使用GLM辅助代码调试、语言润色。所有实验操作、数据、图表、分析和结论均由本人独立完成并核验。本人对提交内容负全部责任。
