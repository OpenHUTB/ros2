import mujoco
import numpy as np
import imageio
import os

# ========== 参数配置 ==========
XML_PATH = os.path.join("model", "model.xml")  # 模型路径
SIM_DURATION = 3        # 仿真时长（秒）
FPS = 30                # 视频帧率
WIDTH, HEIGHT = 640, 400# 渲染分辨率
OUTPUT_DIR = "output"   # 视频输出文件夹
VIDEO_NAME = "sim_video.mp4"
# =============================

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    video_path = os.path.join(OUTPUT_DIR, VIDEO_NAME)

    print(f"[信息] 正在加载模型：{XML_PATH}")
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    writer = imageio.get_writer(video_path, fps=FPS)
    dt_frame = 1.0 / FPS
    next_render_time = 0

    print(f"[信息] 开始渲染，分辨率 {WIDTH}×{HEIGHT}，仿真时长 {SIM_DURATION}s")
    with mujoco.Renderer(model, width=WIDTH, height=HEIGHT) as rend:
        while data.time < SIM_DURATION:
            mujoco.mj_step(model, data)
            if data.time >= next_render_time:
                rend.update_scene(data)
                frame = rend.render()
                writer.append_data(frame)
                next_render_time += dt_frame
    writer.close()
    print(f"\n✅ 完成！视频保存至：{video_path}")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n❌ 发生错误：{e}")