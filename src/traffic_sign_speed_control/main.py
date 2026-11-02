# -*- coding: utf-8 -*-
"""一键入口：python3 main.py <train|evaluate|scenes|demo>

- train     生成合成数据并训练 DigitCNN（输出权重与训练指标）
- evaluate  字符级退化矩阵评测
- scenes    端到端多场景评测（启动真实 ROS 系统）
- demo      以 file 渲染模式运行默认场景并输出 GIF 帧
"""
import subprocess
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent
SCRIPTS = PKG / "scripts"


def sh(cmd):
    print("+", " ".join(str(c) for c in cmd))
    return subprocess.call([str(c) for c in cmd])


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    mode = sys.argv[1]
    if mode == "train":
        sys.exit(sh([sys.executable, SCRIPTS / "train_signs.py"] + sys.argv[2:]))
    if mode == "evaluate":
        sys.exit(sh([sys.executable, SCRIPTS / "evaluate_detector.py"] + sys.argv[2:]))
    if mode == "scenes":
        sys.exit(sh([sys.executable, SCRIPTS / "run_scenes.py"] + sys.argv[2:]))
    if mode == "demo":
        # 离屏渲染帧 + 合成 GIF（需 ffmpeg）。终端 Ctrl+C 会同时送达本进程与
        # launch 子进程：捕获中断，等 launch 退出后用已写入帧合成 GIF。
        import glob
        frames = PKG / "frames"
        gif = PKG / "traffic_sign_speed_control_demo.gif"
        cmd = ["ros2", "launch", "traffic_sign_speed_control",
               "traffic_sign_speed_control.launch.py",
               "render_mode:=file", f"frame_dir:={frames}"]
        print("+", " ".join(cmd))
        proc = subprocess.Popen(cmd)
        try:
            proc.wait()
        except KeyboardInterrupt:
            try:
                proc.wait(timeout=10)
            except Exception:
                proc.kill()
        if not glob.glob(str(frames / "frame_*.png")):
            print("未生成渲染帧（frame_dir 为空），跳过 GIF 合成")
            sys.exit(1)
        sys.exit(sh(["ffmpeg", "-y", "-framerate", "10", "-i",
                     str(frames / "frame_%05d.png"),
                     "-vf", "fps=10,scale=480:480:flags=lanczos,split[a][b];"
                            "[a]palettegen[p];[b][p]paletteuse",
                     str(gif)]))
    print(f"未知模式: {mode}")
    sys.exit(2)


if __name__ == "__main__":
    main()
