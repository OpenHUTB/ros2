#!/usr/bin/env python3
# Save one frame from the DAVE REXROV underwater camera to /tmp/valve_cam_<tag>.png
import sys
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

tag = sys.argv[1] if len(sys.argv) > 1 else "1"
topics = [
    "/rexrov/rexrov/camera/camera_image",
    "/rexrov/rexrov/camera/image_raw",
    "/rexrov/camera/camera_image",
]
rospy.init_node('dave_save_camera', anonymous=True)
msg = None
for t in topics:
    try:
        msg = rospy.wait_for_message(t, Image, timeout=15)
        rospy.loginfo("got image from %s", t)
        break
    except Exception as e:
        rospy.logwarn("no image on %s: %s", t, e)
if msg is None:
    print("NO_CAMERA_IMAGE")
    sys.exit(1)
bridge = cv_bridge.CvBridge()
cv = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
out = "/tmp/valve_cam_%s.png" % tag
cv2.imwrite(out, cv)
print("CAMERA_SAVED %s shape=%s" % (out, cv.shape))
