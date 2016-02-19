#!/usr/bin/env python
#Node that subscribes to joint topic published by the traj_simple_motion node and sends appropriate commands the UR5, this node also publishes actual joint positions for use with other nodez.
import time 
import urx
import logging

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint

#Publisher
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

#Subscriber
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
   listener()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



#Ignore stuff below for the time being


if __name__ == "__main__":
    rob = urx.Robot("192.168.1.6")
    try:
        l = 0.1
        v = 0.07
        a = 0.1
        r = 0.05
        pose = rob.getl()
        pose[2] += l
        rob.movej(pose, acc=a, vel=v, wait=False)
        while True:
            p = rob.getl(wait=True)
            if p[2] > pose[2] - 0.05:
                break

        pose[1] += l 
        rob.movep(pose, acc=a, vel=v, radius=r, wait=False)
        while True:
            p = rob.getl(wait=True)
            if p[1] > pose[1] - 0.05:
                break

        pose[2] -= l
        rob.movep(pose, acc=a, vel=v, radius=r, wait=False)
        while True:
            p = rob.getl(wait=True)
            if p[2] < pose[2] + 0.05:
                break

        pose[1] -= l
        rob.movep(pose, acc=a, vel=v, radius=0, wait=True)

    finally:
        rob.close()
