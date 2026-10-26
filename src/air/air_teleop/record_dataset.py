#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 数据采集脚本：record_dataset（任务四：端到端视觉行为克隆，第一阶段）
# 作用：订阅 /drone/front_camera/image_raw（前视 RGB 图像）与 /drone/cmd_vel
#       （速度指令）。本节点是纯 ROS 节点，不连接 AirSim RPC。
#
# 为什么不用时间戳软同步：geometry_msgs/Twist 不带 Header 时间戳，无法与图像
# 消息做时间对齐（ApproximateTimeSynchronizer 配对失败导致 0 帧）。改为
# “图像触发”模式：
#   1. /drone/cmd_vel 回调：持续缓存最新的速度指令；
#   2. /drone/front_camera/image_raw 回调：收到图像即无条件把该帧与当前缓存
#      的速度指令配对落盘，并在终端强制刷新打印采集计数。
#
# 输出布局（默认在仓库根目录运行）：
#   data/dataset/images/frame_000001.png ...    前视图像（PNG）
#   data/dataset/labels.csv                     列：frame,timestamp,vx,vy,vz,yaw_rate
#   支持多次运行续写：labels.csv 已存在时从最后一帧序号继续编号追加。

import argparse
import csv
import os
import threading

import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class DatasetRecorder:
    """图像触发式采集：图像回调到达即无条件把该帧图像与最新指令落盘"""

    CSV_HEADER = ['frame', 'timestamp', 'vx', 'vy', 'vz', 'yaw_rate']

    def __init__(self, out_dir='data/dataset'):
        self.out_dir = out_dir
        self.images_dir = os.path.join(out_dir, 'images')
        self.csv_path = os.path.join(out_dir, 'labels.csv')
        os.makedirs(self.images_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.frame_idx = 0                          # 累计帧序号（含续写历史）
        self.session_frames = 0                     # 本会话新增帧数
        self.stopped = False

        # 最新指令缓存（由 /drone/cmd_vel 回调写入、图像回调读取）
        self.lock = threading.Lock()
        self.cmd_vel = (0.0, 0.0, 0.0, 0.0)         # (vx, vy, vz, yaw_rate)

        # 打开（或续写）labels.csv，并从最后一帧序号恢复编号
        self.csv_file = self._open_csv()
        self.csv_writer = csv.writer(self.csv_file)

        # 纯 ROS 节点：不连接 AirSim RPC，只订阅图像与指令话题
        self.cmd_sub = rospy.Subscriber('/drone/cmd_vel', Twist,
                                        self.cmd_vel_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('/drone/front_camera/image_raw', Image,
                                          self.image_callback, queue_size=1)
        rospy.on_shutdown(self.stop)

    # ---------------- 落盘 ----------------

    def _open_csv(self):
        """打开 labels.csv：文件已有数据时续写，否则重建并写表头"""
        if os.path.exists(self.csv_path) and os.path.getsize(self.csv_path) > 0:
            self.frame_idx = self._read_last_frame_index()
            csv_file = open(self.csv_path, 'a', newline='')
        else:
            csv_file = open(self.csv_path, 'w', newline='')
            writer = csv.writer(csv_file)
            writer.writerow(self.CSV_HEADER)
            csv_file.flush()
        return csv_file

    def _read_last_frame_index(self):
        """从 labels.csv 最后一条数据行恢复帧序号（无数据行返回 0）"""
        with open(self.csv_path, 'r', newline='') as f:
            rows = list(csv.reader(f))
        for row in reversed(rows):
            if not row or not row[0].startswith('frame_'):
                continue
            try:
                return int(row[0].split('_')[1].split('.')[0])
            except (IndexError, ValueError):
                break
        return 0

    # ---------------- ROS 回调 ----------------

    def cmd_vel_callback(self, msg):
        """缓存最新速度指令"""
        with self.lock:
            self.cmd_vel = (msg.linear.x, msg.linear.y,
                            msg.linear.z, msg.angular.z)

    def image_callback(self, img_msg):
        """图像触发保存：收到图像即无条件落盘，标签取当前最新缓存的指令"""
        with self.lock:
            vx, vy, vz, yaw_rate = self.cmd_vel    # 取当前最新缓存的指令作标签
        try:
            # cv_bridge 直接转成 BGR，便于 cv2.imwrite 落盘
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logerr_throttle(5.0, '图像解码失败：%s', exc)
            return
        self.frame_idx += 1
        frame_name = 'frame_%06d.png' % self.frame_idx
        img_path = os.path.join(self.images_dir, frame_name)
        if not cv2.imwrite(img_path, img):          # 保存失败时跳过，避免产生无图标签行
            rospy.logerr_throttle(5.0, '图像保存失败：%s', img_path)
            return
        self.session_frames += 1
        self.csv_writer.writerow([
            frame_name,
            '%.6f' % img_msg.header.stamp.to_sec(),
            '%.4f' % vx,
            '%.4f' % vy,
            '%.4f' % vz,
            '%.4f' % yaw_rate,
        ])
        self.csv_file.flush()                       # 逐帧落盘，中断也不丢数据
        print(f'\r[已录制 {self.frame_idx} 帧]', end='', flush=True)

    # ---------------- 生命周期 ----------------

    def stop(self):
        """退出时关闭 CSV 文件（可能被 rospy.on_shutdown 与 finally 各调用一次）"""
        if self.stopped:
            return
        self.stopped = True
        self.csv_file.close()
        rospy.loginfo('数据采集结束，累计 %d 帧 -> %s', self.frame_idx, self.csv_path)


def main():
    parser = argparse.ArgumentParser(
        description='任务四数据采集：图像触发保存 /drone/front_camera/image_raw '
                    '与最新 /drone/cmd_vel 指令，落盘到 data/dataset')
    parser.add_argument('--out', default=None, help='数据集输出目录，默认 data/dataset')
    args, _ = parser.parse_known_args()               # 兼容 ROS 参数重映射传入的 _xxx:=yyy

    rospy.init_node('record_dataset')
    # 命令行参数优先，其次读取 ROS 私有参数，最后使用默认值
    out = args.out if args.out is not None else rospy.get_param('~out', 'data/dataset')

    recorder = DatasetRecorder(out_dir=out)
    print('=' * 60)
    print('        任务四数据集采集（端到端视觉行为克隆）')
    print('=' * 60)
    print('  图像输出  : %s' % os.path.abspath(recorder.images_dir))
    print('  标签输出  : %s' % os.path.abspath(recorder.csv_path))
    print('  记录条件  : 收到图像即落盘（无条件）')
    if recorder.frame_idx:
        print('  续写模式  : 从第 %d 帧继续编号' % recorder.frame_idx)
    else:
        print('  续写模式  : 新建数据集（帧号从 1 开始）')
    print('-' * 60)
    print('  先启动 drone_ros_node 与 drone_ros_teleop，起飞后按键遥控；')
    print('  收到图像即自动落盘，按 Ctrl+C 结束采集并输出统计信息')
    print('=' * 60)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        print('-' * 60)
        print('采集统计：本会话新增 %d 帧，累计 %d 帧'
              % (recorder.session_frames, recorder.frame_idx))
        print('-' * 60)


if __name__ == '__main__':
    main()
