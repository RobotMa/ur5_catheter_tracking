#!/usr/bin/env python
import roslib; roslib.load_manifest('active_echo_simul_keyboard')
import rospy

from active_echo_serial.msg import Num 

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

    'u':  1,
    'i': -1,
    'j':  1,
    'k': -1,
    'm':  1,
    ',': -1,
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# member vairables of active_echo_data data type
l_ta = 64
dly  = 2000
tc   = 10

# print the three member variables of active_echo_data
def params(l_ta, dly, tc):
    return "currently:\tl_ta = %s \tdly = %s \ttc = %s" % (l_ta,dly,tc)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('active_echo_data', Num, queue_size = 5)
    rospy.init_node('active_echo_simul_keyboard')

    try:
        print msg
        print params(l_ta, dly, tc)
        while(1):
            
            key = getKey()
            if key in moveBindings.keys():
                if key == 'u' or key == 'i':
                    l_ta = l_ta + moveBindings[key]
                elif key == 'j' or key == 'k':
                    dly   = dly  + moveBindings[key]
                elif key == 'm' or key == ',':
                    tc   = tc   + moveBindings[key]
                else:
                    print 'Not valid key stroke'
            else:
                # l_ta = 0
                # dly  = 0
                # tc   = 0
                if (key == '\x03'):
                    break
            ae_signal = Num()
            ae_signal.l_ta = l_ta; ae_signal.dly = dly; ae_signal.tc = tc 
            pub.publish(ae_signal)

    except:
        print 'Unexpected error in try'

    finally:
        ae_signal = Num()
        ae_signal.l_ta = l_ta; ae_signal.dly = dly; ae_signal.tc = tc
        pub.publish(ae_signal)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
