#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import cv2

distance_to_wall = -1.0
angle_to_wall = 0.0
target = 1
pub = None
buf_big = .2
buf_small = .05
state = "approach"

def scan_received(data):
    global distance_to_wall
    global state

    vals = [(data.ranges[i],data.angle_min+(data.angle_increment*i)) for i in range(len(data.ranges)) if data.ranges[i]>0]
    if len(vals):
        distance_to_wall,angle_to_wall = min(vals)
        if state == "wall follow":
            print "wall follow"
            if abs(distance_to_wall-target)>buf_big:
                state="approach"
                pub.publish(Twist())
            else:
                turn = angle_to_wall-1.57
                pub.publish(Twist(linear=Vector3(x=.05*(4.8-turn)),angular=Vector3(z=turn)))
        elif state == "align":
            print "align"
            if abs(angle_to_wall-1.57) < .2:
                state = "wall follow"
                pub.publish(Twist())
            else:
                pub.publish(Twist(angular=Vector3(z=angle_to_wall-1.57)))
        elif state == "approach":
            if abs(distance_to_wall-target)<buf_small:
                state="align"
                pub.publish(Twist())
            elif angle_to_wall<.2 or angle_to_wall>6.1:
                print "moving"
                pub.publish(Twist(linear=Vector3(x=.4*(distance_to_wall-target))))
            else:
                print "turning"
                pub.publish(Twist(angular = Vector3(z = (-angle_to_wall+3.142))))

def set_target_distance(new_distance):
    """ call back function for the OpenCv Slider to set the target distance """
    global target
    target = new_distance/100.0

def wall_withslider():
    global pub
    """ Main run loop for wall with slider """
    cv2.namedWindow('UI')
    cv2.createTrackbar('distance', 'UI', int(target*100), 300, set_target_distance)
    rospy.init_node('approach_wall', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, scan_received)
    r = rospy.Rate(10)
    while not(rospy.is_shutdown()):
        #if distance_to_wall != -1:
        cv2.waitKey(10)
        r.sleep()

        
if __name__ == '__main__':
    wall_withslider()
