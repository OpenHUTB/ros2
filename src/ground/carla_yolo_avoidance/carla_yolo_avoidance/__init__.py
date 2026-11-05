# -*- coding: utf-8 -*-
"""carla_yolo_avoidance：CARLA + YOLOv8 + ROS 2 Humble 感知与避障示例包。

模块划分（低耦合，便于单独测试）：
    config.py              —— 所有话题名 / 阈值 / 相机参数（全部写死）
    carla_env.py           —— CARLA 与 ultralytics 的导入校验 + 友好报错
    image_utils.py         —— sensor_msgs/Image <-> numpy 互转（不依赖 cv_bridge）
    distance_estimator.py  —— 纯数学函数：由检测框像素宽度估算距离
    detector.py            —— 把 YOLO 原始输出整理成统一检测结果（可单测）
    decision_logic.py      —— 纯函数：由距离得到油门/刹车决策
    carla_camera_node.py   —— 节点①：连接 CARLA、生成车辆与相机、发布原图、执行控制
    yolo_detect_node.py    —— 节点②：YOLOv8 推理、发布检测框与距离
    decision_control_node.py —— 节点③：订阅检测结果、下发油门/刹车
    main.py                —— 统一入口（项目入口文件）
"""

__version__ = '1.0.0'
