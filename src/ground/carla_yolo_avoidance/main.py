# -*- coding: utf-8 -*-
"""统一入口（main.py）

满足课程约定「入口为 main.开头」。三种用法：
    ros2 run carla_yolo_avoidance main --node camera|yolo|decision    # 推荐（setup.py 已注册）
    python3 main.py --node camera                                      # 需先 source install/setup.bash
    ros2 launch carla_yolo_avoidance carla_yolo_avoidance.launch.py   # 一键启动全部节点
"""

from carla_yolo_avoidance.main import main

if __name__ == '__main__':
    main()
