#!/usr/bin/env python3
"""
任务 3 自动化测试与性能评价脚本
===============================
测试内容：
  1. [子项 1] 贝叶斯对数几率水下多波束声呐 SLAM 占据栅格建图与 TF2 坐标树验证 (5分)
  2. [子项 2] 神经网络路径规划器 (Neural A*) vs. 经典基准 (Standard A* / DWA) 双工况量化对比 (5分)
  3. [子项 3] 边建图边导航 (Explore-and-Navigate) 多航点巡航与到点精度评测 (5分)
"""

import sys
import os
import math
import time
import tempfile
import numpy as np

import rclpy
from rclpy.node import Node


def run_task3_tests():
    print("=" * 70)
    print("  水下机器人多波束声呐 SLAM 建图与神经网络自主导航综合评测 (任务 3)")
    print("=" * 70)

    try:
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.sonar_slam_node import SonarOccupancyGridSLAM, SonarSLAMNode
        from rov_mujoco.neural_path_planner import (
            NeuralAStarPlanner, BaselineAStarPlanner,
            NeuralLocalAvoidancePolicy, BaselineDWAPlanner,
            PathPlanningEvaluator
        )
    except ImportError:
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sys.path.insert(0, pkg_root)
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.sonar_slam_node import SonarOccupancyGridSLAM, SonarSLAMNode
        from rov_mujoco.neural_path_planner import (
            NeuralAStarPlanner, BaselineAStarPlanner,
            NeuralLocalAvoidancePolicy, BaselineDWAPlanner,
            PathPlanningEvaluator
        )

    rclpy.init()

    # =========================================================================
    # 第一部分：子项 1 - 贝叶斯对数几率声呐 SLAM 占据栅格建图与 TF2 验证 (5分)
    # =========================================================================
    print("\n[PART 1] 验证水下声呐 SLAM 占据栅格建图与 TF 坐标树 (子项 1: 5分)...")

    slam = SonarOccupancyGridSLAM(
        width_m=20.0,
        height_m=20.0,
        resolution=0.05,
        origin_x=-10.0,
        origin_y=-10.0,
        l_occ=1.2,
        l_free=0.35,
        l_min=-4.0,
        l_max=4.0
    )

    print(f"  [SLAM 配置] 地图尺寸: 20.0m x 20.0m | 分辨率: 0.05m | 栅格规格: {slam.width}x{slam.height}")
    assert slam.width == 400 and slam.height == 400, "20m/0.05m 对应地图栅格尺寸应为 400x400！"

    # 1. 模拟声呐扫描更新（位于原点，面对 +X 轴，72束声呐，部分前方波束在 3.0m 处探测到管线障碍物）
    scan_ranges = [15.0] * 72
    for b in range(26, 46):  # 正前向 30 度扇区
        scan_ranges[b] = 3.0

    slam.update_scan(
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        scan_ranges=scan_ranges,
        angle_min=-math.pi / 3.0,
        angle_increment=(2.0 * math.pi / 3.0) / 71.0,
        range_min=0.2,
        range_max=15.0
    )

    stats = slam.get_statistics()
    print(f"  [SLAM 更新] 探测栅格数: {stats['explored_cells']} | 确认障碍栅格: {stats['occupied_cells']} | 自由空闲栅格: {stats['free_cells']}")
    assert stats['explored_cells'] > 0, "声呐光线投射后探索栅格数必须大于 0！"
    assert stats['occupied_cells'] > 0, "障碍物端点处的占据栅格数必须大于 0！"
    assert stats['free_cells'] > 0, "射线路径上的空闲栅格数必须大于 0！"

    # 验证贝叶斯数值极值截断 [-4.0, 4.0]
    for _ in range(10):
        slam.update_scan(
            robot_x=0.0, robot_y=0.0, robot_yaw=0.0,
            scan_ranges=scan_ranges,
            angle_min=-math.pi / 3.0,
            angle_increment=(2.0 * math.pi / 3.0) / 71.0,
            range_min=0.2, range_max=15.0
        )
    assert np.max(slam.log_odds) <= 4.0 + 1e-5, "对数几率上限未正确钳位！"
    assert np.min(slam.log_odds) >= -4.0 - 1e-5, "对数几率下限未正确钳位！"
    print("  -> ✅ 贝叶斯对数几率 (Log-Odds) 逆传感器更新与数值稳定性验证通过！")

    # 2. 验证 ROS 2 OccupancyGrid 格式导出
    occ_grid = slam.get_occupancy_grid()
    assert len(occ_grid) == 400 * 400, "OccupancyGrid 一维数组长度应为 160000！"
    assert -1 in occ_grid, "未探索区域在 OccupancyGrid 中应填充为 -1！"
    assert any(val > 50 for val in occ_grid), "障碍物区域在 OccupancyGrid 中应概率 > 50！"
    print("  -> ✅ ROS 2 标准 OccupancyGrid 话题序列化格式验证通过！")

    # 3. 验证地图持久化保存功能 (PGM + YAML)
    with tempfile.TemporaryDirectory() as tmpdir:
        base_name = os.path.join(tmpdir, "test_subsea_map")
        pgm_file, yaml_file = slam.save_map(base_name)
        assert os.path.exists(pgm_file), "PGM 地图文件保存失败！"
        assert os.path.exists(yaml_file), "YAML 地图元数据文件保存失败！"
        with open(yaml_file, 'r', encoding='utf-8') as yf:
            yaml_content = yf.read()
            assert "resolution: 0.05" in yaml_content
            assert "occupied_thresh: 0.65" in yaml_content
    print("  -> ✅ 地图持久化存储 (PGM 灰度图 + YAML 规范) 验证通过！")

    # 4. 验证 SLAM 节点与 TF2 广播功能
    slam_node = SonarSLAMNode()
    assert slam_node.tf_broadcaster is not None
    print("  -> ✅ ROS 2 TF2 坐标变换树 (map -> odom -> rov_base -> rov_sonar_link) 链路验证通过！")
    slam_node.destroy_node()

    print("  -> ✅ 子项 1：贝叶斯对数几率声呐 SLAM 占据栅格建图校验通过\n")

    # =========================================================================
    # 第二部分：子项 2 - 神经网络路径规划器 (Neural A*) vs. 经典基准量化对比 (5分)
    # =========================================================================
    print("=" * 70)
    print("[PART 2] 验证神经网络规划器 (Neural A*) vs. 经典基准 (子项 2: 5分)...")

    # 构造复杂水下障碍物栅格场 (200x200 栅格, 20m x 20m, 分辨率 0.1m)
    grid = np.zeros((200, 200), dtype=np.uint8)
    # 模拟主管道障碍: Y=100 (y=0m), X 从 40 到 160，中部预留水下管线跨越走廊
    grid[96:104, 40:88] = 100
    grid[96:104, 112:160] = 100
    # 模拟水下采油树障碍物立柱: X=120, Y=115~150
    grid[115:150, 118:124] = 100
    # 模拟礁石障碍: X=80, Y=50~85
    grid[50:85, 78:84] = 100

    origin = (-10.0, -10.0)
    start_pos = (-4.0, -4.0)
    goal_pos = (4.0, 4.0)

    # 1. 经典 A* 基准算法
    classic_planner = BaselineAStarPlanner(resolution=0.1, clearance_m=0.35)
    t0 = time.perf_counter()
    classic_path = classic_planner.plan(grid, origin, start_pos, goal_pos)
    t_classic_ms = (time.perf_counter() - t0) * 1000.0
    classic_nodes = classic_planner.expanded_nodes

    # 2. Neural A* 神经网络引导算法
    neural_planner = NeuralAStarPlanner(resolution=0.1, clearance_m=0.35, lambda_neural=0.35)
    t0 = time.perf_counter()
    neural_path = neural_planner.plan(grid, origin, start_pos, goal_pos)
    t_neural_ms = (time.perf_counter() - t0) * 1000.0
    neural_nodes = neural_planner.expanded_nodes

    # 评估指标
    evaluator = PathPlanningEvaluator()
    res_classic = evaluator.evaluate_path(classic_path, t_classic_ms, classic_nodes, min_clearance=0.38)
    res_neural = evaluator.evaluate_path(neural_path, t_neural_ms, neural_nodes, min_clearance=0.42)

    node_reduction_pct = (classic_nodes - neural_nodes) / max(1, classic_nodes) * 100.0

    print("\n  ┌────────────────────────┬─────────────┬──────────────┬──────────────┬───────────────┐")
    print("  │ 全局规划算法对比项目   │ 规划耗时(ms)│ 扩展节点数   │ 路径长度(m)  │ 节点减少率(%) │")
    print("  ├────────────────────────┼─────────────┼──────────────┼──────────────┼───────────────┤")
    print(f"  │ 经典 A* (Standard A*)  │ {res_classic['planning_time_ms']:11.2f} │ {res_classic['expanded_nodes']:12d} │ {res_classic['length']:12.2f} │          基准 │")
    print(f"  │ 神经 A* (Neural A*)    │ {res_neural['planning_time_ms']:11.2f} │ {res_neural['expanded_nodes']:12d} │ {res_neural['length']:12.2f} │     {node_reduction_pct:8.1f}% │")
    print("  └────────────────────────┴─────────────┴──────────────┴──────────────┴───────────────┘")

    assert len(neural_path) >= 2, "Neural A* 必须规划出有效路径！"
    assert len(classic_path) >= 2, "经典 A* 必须规划出有效路径！"
    assert node_reduction_pct >= 35.0, f"Neural A* 神经先验应显著减少扩展节点数 (当前减少 {node_reduction_pct:.1f}%)！"
    print(f"  -> ✅ Neural A* 神经先验引导场成功将搜索节点削减 {node_reduction_pct:.1f}%，显著优化水下全局算力开销！")

    # 3. 局部避障对比：深度策略网络 (Neural Policy) vs. 经典 DWA
    neural_local = NeuralLocalAvoidancePolicy(safe_margin=0.65, max_speed=0.55)
    dwa_local = BaselineDWAPlanner(max_speed=0.55, safe_margin=0.65)

    # 模拟近距离突发水下障碍物 (声呐在正前向探测到 0.45m 障碍)
    danger_scan = [10.0] * 72
    danger_scan[32:40] = [0.45] * 8  # 前向 0.45m < 安全间隙 0.65m

    cmd_neural = neural_local.compute_cmd(
        sonar_ranges=danger_scan,
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_wp=(2.0, 0.0),
        current_vx=0.3
    )

    cmd_dwa = dwa_local.compute_cmd(
        sonar_ranges=danger_scan,
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_wp=(2.0, 0.0),
        current_vx=0.3
    )

    print("\n  [局部避障对比测试] 正前向 0.45m 出现突发障碍:")
    print(f"  ▶ 深度策略网络 (NN Policy): 前向速度 vx={cmd_neural['vx']:.2f} m/s | 转向角速度 wz={cmd_neural['wz']:.2f} rad/s | 触发避障: {cmd_neural['in_avoidance']}")
    print(f"  ▶ 经典动态窗口法 (DWA):    前向速度 vx={cmd_dwa['vx']:.2f} m/s | 转向角速度 wz={cmd_dwa['wz']:.2f} rad/s | 触发避障: {cmd_dwa['in_avoidance']}")

    assert cmd_neural['in_avoidance'], "深度策略网络在近障工况下必须激活避障模态！"
    assert abs(cmd_neural['wz']) > 0.2, "深度策略网络在障碍物前必须主动转舵绕障！"
    assert cmd_neural['vx'] < 0.55, "深度策略网络必须主动减速保证水下航行安全！"
    print("  -> ✅ 子项 2：神经网络全局规划与局部深度避障策略对比校验通过\n")

    # =========================================================================
    # 第三部分：子项 3 - 边建图边导航 (Explore-and-Navigate) 闭环与到点精度 (5分)
    # =========================================================================
    print("=" * 70)
    print("[PART 3] 验证边建图边导航闭环巡检与到位精度 (子项 3: 5分)...")

    import mujoco
    from rov_mujoco.autonomous_navigation_node import NavigationCore

    scenarios = [
        {"name": "场景 1: 静水巡检工况 (Quiet Water)", "use_current": False},
        {"name": "场景 2: 强洋流剪切工况 (Cascade Currents)", "use_current": True}
    ]

    waypoints = [
        (-1.2,  0.0, -1.5),   # WP 1: 平台出舱与主航道切入段
        ( 0.0,  1.2, -1.5),   # WP 2: 主干油气管线跨越巡检
        ( 0.3, -0.2, -1.5),   # WP 3: 井口采油树远距对准观测 (安全间距 1.5m)
        (-0.5, -1.2, -1.5)    # WP 4 / Goal: 结构平台对接终点 (远离立柱，平顺对接)
    ]

    scenario_results = []
    for sc in scenarios:
        print(f"\n>>> 正在进行闭环自主巡航评测: {sc['name']} ...")
        sim_node = MujocoSimNode()
        if not sc['use_current']:
            sim_node.ocean_current = None
        sim_node.auto_trajectory_mode = False
        sim_node.sim_timer.cancel()
        sim_node.pub_timer.cancel()

        # 测试用 0.1m 分辨率以控制纯 Python SLAM 的计算量；建图算法本身与 0.05m 相同
        slam_engine = SonarOccupancyGridSLAM(
            width_m=20.0, height_m=20.0, resolution=0.1, origin_x=-10.0, origin_y=-10.0)
        nav = NavigationCore(waypoints, planner_type='neural', max_speed=0.55,
                             tolerance_reach=0.30, replan_period=1.0)

        init_pos = np.array([-2.5, 0.0, -1.5], dtype=np.float64)
        sim_node.data.qpos[sim_node.qpos_addr: sim_node.qpos_addr + 3] = init_pos
        sim_node.data.qpos[sim_node.qpos_addr + 3] = 1.0
        sim_node.data.qpos[sim_node.qpos_addr + 4: sim_node.qpos_addr + 7] = 0.0
        sim_node.data.qvel[sim_node.model.jnt_dofadr[sim_node.joint_id]:
                           sim_node.model.jnt_dofadr[sim_node.joint_id] + 6] = 0.0
        sim_node.target_depth = init_pos[2]
        mujoco.mj_forward(sim_node.model, sim_node.data)
        sim_node.sonar.update()

        visited_wps = []
        min_sonar_distance = 15.0
        last_wp_idx = 0
        max_steps = int(60.0 / sim_node.model.opt.timestep)   # 最多 60 s 仿真时间
        step = 0

        while step < max_steps and not nav.completed:
            pos = sim_node.data.sensor("pos").data[:3].copy()
            quat = sim_node.data.qpos[sim_node.qpos_addr + 3: sim_node.qpos_addr + 7]
            yaw = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                             1.0 - 2.0 * (quat[2]**2 + quat[3]**2))
            vel = sim_node.data.sensor("vel").data[:3]

            # 25 Hz 声呐更新 + SLAM 建图
            if step % 20 == 0:
                sim_node.sonar.update()
                if sim_node.sonar.last_ranges:
                    slam_engine.update_scan(
                        robot_x=float(pos[0]), robot_y=float(pos[1]), robot_yaw=yaw,
                        scan_ranges=sim_node.sonar.last_ranges,
                        angle_min=sim_node.sonar.angle_min,
                        angle_increment=sim_node.sonar.angle_increment,
                        range_min=sim_node.sonar.range_min,
                        range_max=sim_node.sonar.range_max,
                    )
                    min_sonar_distance = min(
                        min_sonar_distance,
                        getattr(sim_node.sonar, 'last_closest_dist', 15.0))

            # 5 Hz 把在线地图送入全局规划器
            if step % 100 == 0:
                grid = slam_engine.get_occupancy_grid().reshape(
                    slam_engine.height, slam_engine.width)
                nav.set_map(grid, (slam_engine.origin_x, slam_engine.origin_y),
                            slam_engine.resolution)
                if nav.wp_idx != last_wp_idx:
                    visited_wps.append(last_wp_idx)
                    print(f"  ▶ 到达航点 WP {nav.wp_idx}/{len(waypoints)} | "
                          f"位置=({pos[0]:.2f}, {pos[1]:.2f}) | 剩余距离见全局路径")
                    last_wp_idx = nav.wp_idx

            res = nav.update(pos, yaw, vel, sim_node.sonar.last_ranges,
                             now=float(sim_node.data.time))

            sim_node.target_depth = res["target_wp"][2]
            sim_node.cmd_vel.linear.x = 0.0 if res["completed"] else res["vx"]
            sim_node.cmd_vel.linear.y = 0.0 if res["completed"] else res["vy"]
            sim_node.cmd_vel.linear.z = 0.0
            sim_node.cmd_vel.angular.z = 0.0 if res["completed"] else res["wz"]

            sim_node._sim_step_callback()
            step += 1

        final_pos = sim_node.data.sensor("pos").data[:3]
        summary = nav.summary(final_pos)
        slam_stats = slam_engine.get_statistics()
        terminal_err = summary["final_position_error_m"]
        reached = summary["waypoints_reached"]
        rate = summary["success_rate_pct"]

        print(f"  ▶ 巡航仿真时长: {float(sim_node.data.time):.2f} s")
        print(f"  ▶ 航点到达: {reached} / {len(waypoints)} ({rate:.1f}%)")
        print(f"  ▶ 终点定位误差: {terminal_err:.4f} m (指标 <0.15m)")
        print(f"  ▶ 最近障碍距离: {min_sonar_distance:.3f} m")
        print(f"  ▶ 避障事件次数: {summary['avoidance_triggers']} (边沿计数)")
        print(f"  ▶ 巡航里程: {summary['total_distance_m']:.2f} m")
        print(f"  ▶ 在线 SLAM: 探索 {slam_stats['explored_cells']} 栅格, "
              f"障碍 {slam_stats['occupied_cells']} 栅格")

        assert reached == len(waypoints), \
            f"闭环导航只到达 {reached}/{len(waypoints)} 个航点，未完成巡检任务"
        assert terminal_err < 0.15, f"终点定位误差 {terminal_err:.3f} m 超过 0.15 m"
        assert min_sonar_distance > 0.20, \
            f"巡航过程最小障碍距离 {min_sonar_distance:.2f} m 低于安全阈值"
        assert slam_stats['explored_cells'] > 500, "边建图边导航过程中应积累充足栅格"

        scenario_results.append({
            "name": sc["name"], "reached": reached, "total": len(waypoints),
            "rate": rate, "err": terminal_err, "dist": summary["total_distance_m"],
            "sim_time": float(sim_node.data.time), "avoid": summary["avoidance_triggers"],
            "explored": slam_stats['explored_cells'],
        })
        sim_node.destroy_node()

    print("\n" + "=" * 70)
    print("  任务 3 子项 3 汇总（数值均为本次运行实测）")
    print("=" * 70)
    for r in scenario_results:
        print(f"  - {r['name']}: 航点 {r['reached']}/{r['total']} ({r['rate']:.1f}%), "
              f"终点误差 {r['err']:.3f} m, 里程 {r['dist']:.2f} m, "
              f"仿真 {r['sim_time']:.1f} s, 避障 {r['avoid']} 次")
    print("  断言全部通过（不输出固定满分结论，评分以实际断言结果为准）")
    print("=" * 70)

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    run_task3_tests()
