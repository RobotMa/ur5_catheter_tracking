#!/usr/bin/env python
import roslib; roslib.load_manifest('active_echo_simul_keyboard')
import rospy
import tf

from geometry_msgs.msg import Point
from active_echo_serial.msg import Num 
from math import sin 

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to active_echo_data!
---------------------------
Moving around:
u    i
j    k
m    ,

q/z : increase/decrease l_ta changing speed by 10%
w/x : increase/decrease dly  changing speed by 10%
e/c : increase/decrease tc   changing speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {

    'u':  0.0002,
    'i': -0.0002,
    'j':  0.0002,
    'k': -0.0002,
    'm':  0.0002,
    ',': -0.0002,
    'o':  0.000,
    'a': [], # accelerate x step length
    'd': [], # reset x step length
    'h': [], # straight line 
    's': [], # sine wave line
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# initial position of the active echo point
x = 0.170 # m
y = 0.425 # m
z = 0.048 # m
count = 0

# print the three member variables of active_echo_data
def params(x, y, z):
    return "Initial position of the AE elmemnt is:\tx = %s \ty = %s \tz = %s" % (x,y,z)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('active_echo_signal_simul_keyboard')
    pub = rospy.Publisher('active_echo_position_topic', Point, queue_size = 5)
    br = tf.TransformBroadcaster() 
    rate = rospy.Rate(50) 
    angle = 0
    step = 1

    try:
        print params(x, y, z)
        while(1):

            key = getKey()
            point = Point()
            if key in moveBindings.keys():
                if key == 'u' or key == 'i':
                    x = x + step*moveBindings[key]
                elif key == 'j' or key == 'k':
                    y = y + step*moveBindings[key]
                elif key == 'm' or key == ',':
                    z = z + step*moveBindings[key]
                elif key == 'a':
                    step = 5*step
                elif key == 'd':
                    step = 1
                elif key == 'o':
                    print "AE element static"
                elif key == 'h':
                    while not rospy.is_shutdown():
                        x = x + 0.0002
                        point.x = x 
                        point.y = y
                        point.z = z
                        pub.publish(point)
                        rate.sleep()
                        if (key == '\x03'):
                            break
                elif key == 's':
                    y_cur = y
                    while not rospy.is_shutdown():
                        x = x + 0.0002
                        y = y_cur + 0.01*sin(angle)
                        point.x = x 
                        point.y = y
                        point.z = z
                        pub.publish(point)
                        angle = angle + 0.02
                        print "angle is %s" % angle
                        rate.sleep()
                        if (key == '\x03'):
                            break

                else:
                    print 'Not valid key stroke'
            else:
                if (key == '\x03'):
                    break

            point.x = x
            point.y = y
            point.z = z

            count = count + 1
            print "count is %s" % count
            pub.publish(point)

            br.sendTransform((x,y,z), 
                             tf.transformations.quaternion_from_euler(0,0,0),
                             rospy.Time.now(),
                             "active_echo_position",
                             "world")
            rate.sleep()
    except:
        print 'Unexpected error in try'

    finally:
        br.sendTransform((x,y,z), 
                         tf.transformations.quaternion_from_euler(0,0,0),
                         rospy.Time.now(),
                         "active_echo_position",
                         "world")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
