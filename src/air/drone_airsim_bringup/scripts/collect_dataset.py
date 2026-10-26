#!/usr/bin/env python3
"""Collect drone camera frames into a YOLO-format dataset.

Subscribes /drone/camera/image_raw, saves JPEG frames and optional YOLO labels.
With ~auto_label:=true it uses a pre-trained YOLOv8 model to create
pseudo-labels (car=0, person=1). Pseudo-labels are a starting point; replace
them with manual or CARLA ground-truth labels before treating metrics as real.
"""

import os
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

COCO_CAR = 2
COCO_PERSON = 0
CLASS_NAMES = ["car", "person"]


class DatasetCollector(object):
    def __init__(self):
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/.ros/drone_dataset")
        )
        self.save_rate = float(rospy.get_param("~save_rate", 2.0))
        self.max_images = int(rospy.get_param("~max_images", 200))
        self.auto_label = bool(rospy.get_param("~auto_label", False))
        self.model_path = rospy.get_param("~model", "yolov8n.pt")
        self.conf = float(rospy.get_param("~conf", 0.25))

        self.images_dir = os.path.join(self.output_dir, "images")
        self.labels_dir = os.path.join(self.output_dir, "labels")
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.labels_dir, exist_ok=True)
        self.write_dataset_yaml()

        self.model = None
        if self.auto_label:
            from ultralytics import YOLO

            self.model = YOLO(self.model_path)
            rospy.loginfo("auto-label model: %s", self.model_path)

        self.count = 0
        self.last_save = 0.0
        self.sub = rospy.Subscriber(
            "/drone/camera/image_raw", Image, self.image_cb, queue_size=1
        )
        rospy.loginfo("saving to %s at %.1f Hz", self.output_dir, self.save_rate)

    def write_dataset_yaml(self):
        with open(os.path.join(self.output_dir, "classes.txt"), "w") as f:
            f.write("\n".join(CLASS_NAMES) + "\n")
        with open(os.path.join(self.output_dir, "data.yaml"), "w") as f:
            f.write("path: %s\n" % self.output_dir)
            f.write("train: images\n")
            f.write("val: images\n")
            f.write("names:\n")
            for i, name in enumerate(CLASS_NAMES):
                f.write("  %d: %s\n" % (i, name))

    def image_cb(self, msg):
        if self.count >= self.max_images:
            return
        now = time.time()
        if now - self.last_save < 1.0 / max(self.save_rate, 0.1):
            return
        self.last_save = now

        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "rgb8":
            arr = arr.reshape(msg.height, msg.width, 3)
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        else:
            arr = arr.reshape(msg.height, msg.width, 3)

        stem = "frame_%06d" % self.count
        image_path = os.path.join(self.images_dir, stem + ".jpg")
        cv2.imwrite(image_path, arr)

        lines = []
        if self.model is not None:
            result = self.model.predict(arr, conf=self.conf, verbose=False)[0]
            if result.boxes is not None:
                for box, cls in zip(result.boxes.xyxy, result.boxes.cls):
                    coco = int(cls.item())
                    if coco == COCO_CAR:
                        label = 0
                    elif coco == COCO_PERSON:
                        label = 1
                    else:
                        continue
                    x1, y1, x2, y2 = [float(v) for v in box]
                    xc = (x1 + x2) / 2.0 / msg.width
                    yc = (y1 + y2) / 2.0 / msg.height
                    w = (x2 - x1) / msg.width
                    h = (y2 - y1) / msg.height
                    lines.append("%d %.6f %.6f %.6f %.6f" % (label, xc, yc, w, h))

        with open(os.path.join(self.labels_dir, stem + ".txt"), "w") as f:
            f.write("\n".join(lines))

        self.count += 1
        if self.count % 10 == 0:
            rospy.loginfo("saved %d images", self.count)
        if self.count >= self.max_images:
            rospy.loginfo("reached max_images=%d", self.max_images)


if __name__ == "__main__":
    rospy.init_node("drone_dataset_collector")
    DatasetCollector()
    rospy.spin()
