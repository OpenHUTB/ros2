AEB 实验摘要
地图: Carla/Maps/Town10HD_Opt
决策依据: 激光雷达观测（真值仅用于评估对账）
初始距离(首帧真值): 56.0 m（配置 lead_gap=55.0 m，实际按路段取整）
制动触发时刻: 4.36 s
制动后停稳时刻: 6.11 s
激光雷达有效检出帧: 9 / 99（语义实例点数 >= 4 才成目标；远距稀疏会漏检，决策按锁存与最后一次有效测量处理）
停稳时激光雷达测距（最后一次有效测量，到前车最近表面）: 3.55 m
停稳时真值距离（中心距，仅评估）: 4.49 m
全程最近距离(真值): 4.49 m
传感器失效帧数: 0（失效安全策略：点云流超过 1.0 s 未更新即判故障并制动；「走廊内暂时无目标」不算故障）
关键帧: frame_approach_000.png, frame_approach_001.png, frame_approach_002.png, frame_approach_003.png, frame_approach_004.png, frame_brake_005.png, frame_brake_006.png, frame_stopped_007.png, frame_stopped_008.png
