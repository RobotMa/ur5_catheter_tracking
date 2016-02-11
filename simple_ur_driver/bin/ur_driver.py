#!/usr/bin/env python
# ROS IMPORTS
import roslib; roslib.load_manifest('simple_ur_driver')
import rospy
import tf; import tf_conversions as tf_c
import PyKDL
# MSGS and SERVICES
from simple_ur_msgs.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import *
import time
# URX Universal Robot Driver
import urx
# OTHER
import logging

def callback(self,data):
    self.servo_enable()
    a = 0.1
    v = 0.07
    pose[0] = data.positon[0]
    pose[1] = data.positon[1]
    pose[2] = data.positon[2]
    pose[3] = data.positon[3]
    pose[4] = data.positon[4]
    pose[5] = data.positon[5]
    self.rob.movej(pose,acc=a,vel=v,wait=False)
    self.service_disable()

class URDriver():
    MAX_ACC = 1.0
    MAX_VEL = 1.8
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    MSG_QUIT = 0
    MSG_MOVEL=1
    MULT_jointstate = 10000.0
    MULT_time = 1000000.0
    MULT_blend = 1000.0
    
 


    def __init__(self):
        rospy.init_node('ur_driver',anonymous=True)
        rospy.logwarn('SIMPLE_UR DRIVER LOADING')
        # TF
        self.broadcaster_ = tf.TransformBroadcaster()
        self.listener_ = tf.TransformListener()
        # SERVICES
        self.set_stop_service = rospy.Service('simple_ur_msgs/SetStop', SetStop, self.set_stop_call)
        self.set_servo_mode_service = rospy.Service('simple_ur_msgs/SetServoMode', SetServoMode, self.set_servo_mode_call)
        # PUBLISHERS AND SUBSCRIBERS
        self.driver_status_publisher = rospy.Publisher('/ur_robot/driver_status',String)
        self.robot_state_publisher = rospy.Publisher('/ur_robot/robot_state',String)
        self.joint_state_publisher = rospy.Publisher('joint_states',JointState)
        self.desired_joint_state_subscriber = rospy.Subscriber('joint_trajectory_real', JointState,callback)

        ### Set Up Robot ###
        #self.rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)
        self.rob = urx.Robot("192.168.1.155")
        if not self.rob:
            rospy.logwarn('SIMPLE UR  - ROBOT NOT CONNECTED')
            self.driver_status = 'DISCONNECTED'
            self.robot_state = 'POWER OFF'
        else:
            rospy.logwarn('SIMPLE UR - ROBOT CONNECTED SUCCESSFULLY')
            self.rtm = self.rob.get_realtime_monitor()
            rospy.logwarn('SIMPLE UR - GOT REAL TIME INTERFACE TO ROBOT')        
            self.driver_status = 'IDLE'

        ### Set Up PID ###


        while not rospy.is_shutdown():
            self.update()
            self.check_driver_status()
            self.check_robot_state()
            self.publish_status()
            rospy.spin()
            rospy.sleep(.01)
  
        # Finish
        rospy.logwarn('SIMPLE UR - ROBOT INTERFACE CLOSING')
        self.rob.shutdown()

    def update(self):
        if not self.driver_status == 'DISCONNECTED':
            # Get Joint Positions
            self.current_joint_positions = self.rob.getj()
            msg = JointState()
            msg.header.stamp = rospy.get_rostime()
            msg.header.frame_id = "robot_secondary_interface_data"
            msg.name = self.JOINT_NAMES
            msg.position = self.current_joint_positions
            msg.velocity = [0]*6
            msg.effort = [0]*6
            self.joint_state_publisher.publish(msg)
            
            # Get TCP Position
            tcp_angle_axis = self.rob.getl()
            # Create Frame from XYZ and Angle Axis
            T = PyKDL.Frame()   
            axis = PyKDL.Vector(tcp_angle_axis[3],tcp_angle_axis[4],tcp_angle_axis[5])
            # Get norm and normalized axis
            angle = axis.Normalize()
            # Make frame
            T.p = PyKDL.Vector(tcp_angle_axis[0],tcp_angle_axis[1],tcp_angle_axis[2])
            T.M = PyKDL.Rotation.Rot(axis,angle)
            # Create Pose
            self.current_tcp_pose = tf_c.toMsg(T)
            self.current_tcp_frame = T
            self.broadcaster_.sendTransform(tuple(T.p),tuple(T.M.GetQuaternion()),rospy.Time.now(), '/endpoint','/base_link')


    def check_driver_status(self):
        if self.driver_status == 'DISCONNECTED':
            pass
        elif self.driver_status == 'IDLE': 
            pass
        elif self.driver_status == 'SERVO': 
            pass
        elif self.driver_status == 'FOLLOW': 
            pass
        elif self.driver_status == 'TEACH': 
            pass

    def set_servo_mode_call(self,req):
        if self.driver_status == 'TEACH':
            rospy.logwarn('SIMPLE UR -- cannot enter servo mode, teach mode is active')
            return 'FAILED - teach mode is active'
        else:
            rospy.logwarn('MODE IS ['+req.mode+']')
            if req.mode == 'SERVO':
                self.driver_status = 'SERVO'
                return 'SUCCESS - servo mode enabled'
            elif req.mode == 'FOLLOW':
                self.driver_status = 'FOLLOW'
                return 'SUCCESS - follow mode enabled'
            elif req.mode == 'DISABLE':
                self.driver_status = 'IDLE'
                return 'SUCCESS - servo mode disabled'

    def set_stop_call(self,req):
        rospy.logwarn('SIMPLE UR - STOPPING ROBOT')
        self.rob.stop()
        return 'SUCCESS - stopped robot'

    def publish_status(self):
        self.driver_status_publisher.publish(String(self.driver_status))
        self.robot_state_publisher.publish(String(self.robot_state))

    def check_robot_state(self):
        mode = self.rob.get_all_data()['RobotModeData']

        if not mode['isPowerOnRobot']:
            self.robot_state = 'POWER OFF'
            return

        if mode['isEmergencyStopped']:
            self.robot_state = 'E-STOPPED'
            self.driver_status = 'IDLE - WARN'
        elif mode['isSecurityStopped']:
            self.robot_state = 'SECURITY STOP'
            self.driver_status = 'IDLE - WARN'
        elif mode['isProgramRunning']:
            self.robot_state ='RUNNING PROGRAM'
        else:
            self.robot_state = 'RUNNING IDLE'

def servo_enable(self):
        if self.driver_status == 'IDLE':
            try:
                rospy.wait_for_service('/simple_ur_msgs/SetServoMode',2)
            except rospy.ROSException as e:
                print 'Could not find SetServoMode service'
                return
            try:
                servo_mode_service = rospy.ServiceProxy('/simple_ur_msgs/SetServoMode',SetServoMode)
                result = servo_mode_service('SERVO')
                rospy.logwarn(result.ack)
                # self.servo = 'SERVO'               
            except rospy.ServiceException, e:
                print e
        else:
            rospy.logwarn('FAILED, driver is in ['+self.driver_status+'] mode.')

def servo_disable(self):
        if self.driver_status == 'SERVO' or self.driver_status == 'FOLLOW':
            try:
                rospy.wait_for_service('/simple_ur_msgs/SetServoMode',2)
            except rospy.ROSException as e:
                print 'Could not find SetServoMode service'
                return
            try:
                servo_mode_service = rospy.ServiceProxy('/simple_ur_msgs/SetServoMode',SetServoMode)
                result = servo_mode_service('DISABLE')
                rospy.logwarn(result.ack)
            except rospy.ServiceException, e:
                print e
        else:
            rospy.logwarn('FAILED, driver is in ['+self.driver_status+'] mode.')



if __name__ == "__main__":
    robot_driver = URDriver()


