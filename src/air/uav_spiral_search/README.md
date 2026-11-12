# uav_spiral_search — 基于阿基米德螺旋的无人机自主搜索与目标定位

## 运行环境

- Windows 10/11 + OpenHUTB（AIR）仿真器，RPC 端口 `41451` 可达
- Ubuntu 20.04（虚拟机/WSL2）+ ROS 2 Foxy/Galactic
- Python 依赖：`rclpy`、`numpy`、`opencv-python`（随 ROS 2 自带 `python3-opencv`）
- 复用 `openhutb_yolo_mlp_control` 功能包中的 `airsim_bridge` 桥接节点，需先构建该包

## 运行步骤

1. 在 Windows 端启动 OpenHUTB 仿真器，等待场景加载完成。

2. 在 ROS 2 端构建并加载工作空间：

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select openhutb_yolo_mlp_control uav_spiral_search
   source install/setup.bash
   ```

3. 预览模式（不发送飞行指令），观察状态机与检测效果：

   ```bash
   ros2 launch uav_spiral_search spiral_search.launch.py \
     airsim_ip:="$AIRSIM_IP" dry_run:=true show_window:=true
   ```

   另开终端查看任务状态（JSON）与预览速度：

   ```bash
   ros2 topic echo /openhutb/spiral_search/status
   ```

4. 确认检测颜色与相机方向无误后，实际执行搜索：

   ```bash
   ros2 launch uav_spiral_search spiral_search.launch.py \
     airsim_ip:="$AIRSIM_IP" dry_run:=false show_window:=true
   ```

   无人机自动爬升到搜索高度（`search_height`），沿阿基米德螺旋外扩扫描；连续 `confirm_frames` 帧检测到目标颜色即锁定，飞往目标估计位置上方悬停并输出估计坐标；若螺旋扩展到 `max_radius` 仍未发现目标则原地悬停。任务结束后由操作者在仿真器中接管降落。

5. （可选）运行单元测试与离线闭环仿真：

   ```bash
   python3 -m pytest test/
   python3 tools/simulate_mission.py --target-x 12 --target-y -14 --plot sim.png
   ```

详细参数与状态说明见 `docs/air/uav_spiral_search/spiral_search.md`。
