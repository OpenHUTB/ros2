#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 单足弹跳机器人简单仿真
就是个一根腿的小机器人，在地上原地一蹲一跳
"""

import mujoco
import mujoco.viewer
import numpy as np
import sys

def main():
    model_path = sys.argv[1] if len(sys.argv) > 1 else "../models/hopper.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    print("=" * 50)
    print("MuJoCo 单足弹跳机器人简单控制")
    print("=" * 50)
    print("小机器人在地上原地一蹲一跳")
    print("=" * 50)
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        step = 0
        while viewer.is_running():
            t = data.time
            
            # 正弦力控制：周期性压缩腿部然后弹开，力调大一点真的能跳起来
            force = 150 * np.sin(t * 3)
            data.ctrl[0] = force
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            step += 1
            if step % 200 == 0:
                height = data.sensordata[1]
                print(f"\r当前高度: {height:.2f}m", end="")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n退出控制")
