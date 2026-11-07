#!/usr/bin/env python3
"""PilotNet 推理节点

订阅 camera/image_raw，用 PilotNet CNN 推理，
发布 pilotnet/control（std_msgs/Float32MultiArray: [steering, throttle]）。
注意：launch 启动时带有 group ns，实际话题为 /carla_pilotnet_ros/...。

图像预处理：BGR -> RGB，resize 到 (input_w, input_h)，归一化到 [0,1]。
输入图像尺寸不匹配时会自动 resize，但编码必须是 bgr8。
"""
import os
import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


try:
    import torch
except ImportError as e:
    print("[ERROR] 无法导入 PyTorch: " + str(e))
    print("[ERROR] 请安装 torch：pip install torch==2.0.1 --index-url https://download.pytorch.org/whl/cpu")
    sys.exit(1)

try:
    from model import PilotNet
except ImportError as e:
    print("[ERROR] 无法导入 model.py: " + str(e))
    sys.exit(1)


class PilotNetInference:
    def __init__(self):
        rospy.init_node('pilotnet_inference', anonymous=False)

        model_path = rospy.get_param('~model_path', 'pilotnet.pth')
        self.input_h = rospy.get_param('~input_h', 66)
        self.input_w = rospy.get_param('~input_w', 200)
        self.infer_every_n = rospy.get_param('~infer_every_n', 2)

        # 权重路径解析：先模块目录，再当前目录
        if not os.path.isabs(model_path):
            candidates = [
                os.path.join(pkg_root, model_path),
                os.path.join(os.getcwd(), model_path),
            ]
            for c in candidates:
                if os.path.exists(c):
                    model_path = c
                    break

        self.model = PilotNet(self.input_h, self.input_w)
        if os.path.exists(model_path):
            try:
                self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
                rospy.loginfo("Loaded weights: " + model_path)
            except Exception as e:
                rospy.logerr("权重加载失败: " + str(e))
                rospy.logerr("将使用随机初始化权重（仅用于演示 ROS 话题链路）")
        else:
            rospy.logwarn("未找到权重文件: " + model_path)
            rospy.logwarn("将使用随机初始化权重（仅用于演示 ROS 话题链路，无实际控制效果）")
            rospy.logwarn("请确认本模块根目录下包含 pilotnet.pth（项目中已提供）")
        self.model.eval()

        self.frame_count = 0
        self.received_first_image = False

        # 话题命名与 launch 中的 group ns 对应
        self.ctrl_pub = rospy.Publisher('pilotnet/control', Float32MultiArray, queue_size=1)
        self.annotated_pub = rospy.Publisher('pilotnet/annotated_image', Image, queue_size=1)
        rospy.Subscriber('camera/image_raw', Image, self.on_image, queue_size=1)

        # 订阅超时监控：5 秒后还没收到图像就提示
        rospy.Timer(rospy.Duration(5.0), self._check_subscription)

        rospy.loginfo("PilotNet inference node ready")
        rospy.loginfo("等待 /camera/image_raw 图像...")

    def _check_subscription(self, event):
        if not self.received_first_image:
            rospy.logwarn("启动后 5 秒未收到 /camera/image_raw 图像")
            rospy.logwarn("请检查 image_publisher 节点是否正常运行，或话题名是否匹配")

    @staticmethod
    def imgmsg_to_cv2(msg):
        n_channels = 3 if msg.encoding in ('bgr8', 'rgb8') else 1
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, n_channels)

    @staticmethod
    def cv2_to_imgmsg(frame):
        msg = Image()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = 3 * frame.shape[1]
        msg.data = frame.tobytes()
        return msg

    def on_image(self, msg):
        self.received_first_image = True

        try:
            frame = self.imgmsg_to_cv2(msg)
        except Exception as e:
            rospy.logerr("图像解码失败: " + str(e))
            return

        self.frame_count += 1
        if self.frame_count % self.infer_every_n != 0:
            return

        # 预处理：BGR -> RGB，resize 到网络输入尺寸，归一化到 [0,1]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.input_w, self.input_h))
        tensor = torch.from_numpy(resized).float().permute(2, 0, 1).unsqueeze(0) / 255.0

        try:
            with torch.no_grad():
                out = self.model(tensor)
        except Exception as e:
            rospy.logerr("推理失败: " + str(e))
            return

        steering = float(out[0, 0].item())
        throttle = float(out[0, 1].item())

        ctrl_msg = Float32MultiArray()
        ctrl_msg.data = [steering, throttle]
        self.ctrl_pub.publish(ctrl_msg)

        rospy.loginfo_throttle(1.0,
            "steering=%.3f throttle=%.3f" % (steering, throttle))

        annotated = frame.copy()
        cv2.putText(annotated,
                    "steer=%.2f thr=%.2f" % (steering, throttle),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0), 2)
        out_msg = self.cv2_to_imgmsg(annotated)
        out_msg.header = msg.header
        self.annotated_pub.publish(out_msg)


if __name__ == '__main__':
    try:
        PilotNetInference()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
