#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3


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
        linear = Vector3(x=0.0,y=0.0,z=0.0)
        angular = Vector3(x=0.0,y=0.0,z=0.0)
        if key == 'j':
            angular.z = 0.5
        elif key =='l':
            angular.z = 0.5
        rospy.loginfo([linear,angular])
        pub.publish(Twist(linear = linear,angular = angular))
        
        r.sleep()
        
if __name__ == '__main__':
    try:
        rc()
    except rospy.ROSInterruptException: pass
