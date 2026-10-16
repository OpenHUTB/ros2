import mujoco
import mujoco.viewer
import time

def main():
    # 定义模型
    model = mujoco.MjModel.from_xml_string("""
        <mujoco>
        <option gravity = "0 0 -10"/>
            <worldbody>
                <light pos = "0 0 5"/>
                <geom type = "plane" size = "2 2 0.1" rgba = ".9 .9 .9 1"/>
                    <body pos = "0 0 4">
                    <freejoint/>
                    <geom type = "sphere" size = "0.1" rgba = ".5 .5 .5 2" solref = "0.01 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    """)

    # 创建数据对象
    data = mujoco.MjData(model)

    # 启动循环
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time < 10):

            mujoco.mj_step(model, data)

            viewer.sync()

            time.sleep(model.opt.timestep)

if __name__ == '__main__':
    main()