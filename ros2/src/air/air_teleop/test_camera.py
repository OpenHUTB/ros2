#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 相机出图单点验证脚本：test_camera（约 10 行核心逻辑，不依赖 ROS）
# 作用：连上指定 host 的 AirSim，抓取一帧相机图像并打印通道数与 shape，
#       用于验证 AirSim 本身能否正常出图（排查 /drone/front_camera/image_raw 无数据问题）。

import argparse

import airsim
import numpy as np


def main():
    parser = argparse.ArgumentParser(description='AirSim 相机出图单点验证')
    parser.add_argument('--host', default='127.0.0.1', help='宿主机（AirSim）IP')
    parser.add_argument('--camera', default='front_center', help='相机名，默认 front_center')
    parser.add_argument('--vehicle', default='',
                        help='载具名称，空串表示通过 listVehicles 自动获取（回退 SimpleFlight）')
    args = parser.parse_args()

    client = airsim.MultirotorClient(ip=args.host)
    client.confirmConnection()
    # 载具名称：显式传入优先，否则 listVehicles 自动获取第一架载具
    vehicle_name = args.vehicle
    if not vehicle_name:
        try:
            vehicles = client.listVehicles()
        except Exception as exc:
            print('[警告] listVehicles 查询失败：%s，回退默认载具名 SimpleFlight' % exc)
            vehicles = []
        vehicle_name = vehicles[0] if vehicles else 'SimpleFlight'
    print('使用载具: %s' % vehicle_name)
    resp = client.simGetImages([
        airsim.ImageRequest(args.camera, airsim.ImageType.Scene,
                            pixels_as_float=False, compress=False)],
        vehicle_name=vehicle_name)[0]
    img = np.frombuffer(resp.image_data_uint8,
                        dtype=np.uint8).reshape(resp.height, resp.width, 3)
    print('相机 %s 出图正常：shape %s，通道数 %d'
          % (args.camera, img.shape, img.shape[2]))


if __name__ == '__main__':
    main()
