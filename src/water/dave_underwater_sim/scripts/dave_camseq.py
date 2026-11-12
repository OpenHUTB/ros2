#!/usr/bin/env python3
# Save a time-gated sequence of underwater camera frames to /tmp/camseq for GIF creation.
import os
import time
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

OUT = "/tmp/camseq"
os.makedirs(OUT, exist_ok=True)
for f in os.listdir(OUT):
    os.remove(os.path.join(OUT, f))

DURATION = 12.0
INTERVAL = 0.2
bridge = cv_bridge.CvBridge()
count = 0
last = 0.0


def cb(msg):
    global count, last
    now = time.time()
    if now - last < INTERVAL:
        return
    last = now
    try:
        cv = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(os.path.join(OUT, "f_%03d.png" % count), cv)
        count += 1
    except Exception as e:
        rospy.logwarn("frame save failed: %s", e)


rospy.init_node('dave_camseq', anonymous=True)
rospy.Subscriber('/rexrov/rexrov/camera/camera_image', Image, cb, queue_size=2)
t0 = time.time()
r = rospy.Rate(10)
while not rospy.is_shutdown() and time.time() - t0 < DURATION:
    r.sleep()
print("CAMSEQ_DONE frames=%d" % count, flush=True)
