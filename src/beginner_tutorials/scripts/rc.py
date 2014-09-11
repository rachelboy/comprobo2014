#!/usr/bin/env python
import rospy
from geometry_msgs import Twist


def getch():
    """ Return the next character typed on the keyboard """
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def rc():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('rc', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        key = getch()
        if key == 'j':
            rospy.loginfo([[0.0,0.0,0.0],[0.0,0.0,0.5]])
            pub.publish([[0.0,0.0,0.0],[0.0,0.0,0.5]])
        elif key =='k':
            rospy.loginfo([[0.0,0.0,0.0],[0.0,0.0,0.0]])
            pub.publish([[0.0,0.0,0.0],[0.0,0.0,0.0]])
        elif key =='l':
            rospy.loginfo([[0.0,0.0,0.0],[0.0,0.0,-0.5]])
            pub.publish([[0.0,0.0,0.0],[0.0,0.0,-0.5]])
        
        r.sleep()
        
if __name__ == '__main__':
    try:
        rc()
    except rospy.ROSInterruptException: pass
