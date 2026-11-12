#!/usr/bin/env python3
# DAVE oberon7 arm motion demo: PD effort control to move the 6-DOF arm, then gripper open/close.
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

ARM_JOINTS = ["azimuth", "shoulder", "elbow", "roll", "pitch", "wrist"]
TARGETS = {"azimuth": 0.5, "shoulder": 1.0, "elbow": -1.0,
           "roll": 0.2, "pitch": 0.5, "wrist": 0.2}

rospy.init_node('dave_arm_move_demo', anonymous=True)

pub = {}
for j in ARM_JOINTS:
    pub[j] = rospy.Publisher('/rexrov/oberon7/%s/controller/command' % j,
                             Float64, queue_size=1)
pub["finger_left"] = rospy.Publisher('/rexrov/oberon7/finger_left_joint/controller/command',
                                     Float64, queue_size=1)
pub["finger_right"] = rospy.Publisher('/rexrov/oberon7/finger_right_joint/controller/command',
                                      Float64, queue_size=1)

pos = {}
vel = {}


def cb(msg):
    for n, p, v in zip(msg.name, msg.position, msg.velocity):
        if n.startswith('oberon7/'):
            key = n.split('/', 1)[1]
            pos[key] = p
            vel[key] = v


sub = rospy.Subscriber('/rexrov/joint_states', JointState, cb)

# wait for joint states
t0 = time.time()
while not pos and time.time() - t0 < 20 and not rospy.is_shutdown():
    time.sleep(0.1)
rospy.loginfo("arm init pos: %s", {j: round(pos.get(j, 0), 3) for j in ARM_JOINTS})

rate = rospy.Rate(50)
start = time.time()
while time.time() - start < 10 and not rospy.is_shutdown():
    for j, target in TARGETS.items():
        if j in pos:
            err = target - pos[j]
            cmd = 400.0 * err - 20.0 * vel.get(j, 0.0)
            cmd = max(-300.0, min(300.0, cmd))
            pub[j].publish(Float64(cmd))
    rate.sleep()

for j in ARM_JOINTS:
    pub[j].publish(Float64(0.0))
rospy.loginfo("arm moved pos: %s", {j: round(pos.get(j, 0), 3) for j in ARM_JOINTS})

# gripper: close (both fingers press inward), then open
for i in range(150):
    pub["finger_left"].publish(Float64(12.0))
    pub["finger_right"].publish(Float64(-12.0))
    rate.sleep()
rospy.loginfo("gripper closed pos: L=%.4f R=%.4f", pos.get("finger_left_joint", 0.0), pos.get("finger_right_joint", 0.0))
for i in range(150):
    pub["finger_left"].publish(Float64(-12.0))
    pub["finger_right"].publish(Float64(12.0))
    rate.sleep()
rospy.loginfo("gripper opened pos: L=%.4f R=%.4f", pos.get("finger_left_joint", 0.0), pos.get("finger_right_joint", 0.0))
print("ARM_MOVE_DONE", flush=True)
