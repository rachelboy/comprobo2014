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

class WallFollower():
    def __init__(self, target=1, buf_big=.35, buf_small=.05):
        self.distance_to_wall = -1.0
        self.angle_to_wall = 0.0
        self.target = target
        self.buf_big = buf_big
        self.buf_small = buf_small
        self.state = "approach"

        self.turn = 0.0
        self.speed = 0.0

        rospy.init_node('approach_wall', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)

    def switch_state(self,state):
        self.state=state
        self.turn, self.speed = 0.0,0.0

    def scan_received(self,data):
        vals = [(data.ranges[i],data.angle_min+(data.angle_increment*i)) for i in range(len(data.ranges)) if data.ranges[i]>0]
        if len(vals):
            self.distance_to_wall,self.angle_to_wall = min(vals)
            if self.state == "wall follow":
                print "wall follow"
                if abs(self.distance_to_wall-self.target)>self.buf_big:
                    self.switch_state("approach")
                else:
                    self.turn = self.angle_to_wall-1.57
                    self.speed = .05*(4.8-self.turn)
            elif self.state == "align":
                print "align"
                if abs(self.angle_to_wall-1.57) < .2:
                    self.switch_state("wall follow")
                else:
                    self.turn = .7*(self.angle_to_wall-1.57)
                    self.speed = 0.0
            elif self.state == "approach":
                if abs(self.distance_to_wall-self.target)<self.buf_small:
                    self.switch_state("align")
                elif self.angle_to_wall<.2 or self.angle_to_wall>6.1:
                    print "moving"
                    self.turn = 0.0
                    self.speed = .4*(self.distance_to_wall-self.target)
                else:
                    print "turning"
                    self.turn = .2*(-self.angle_to_wall+3.142)
                    self.speed = 0.0

    def set_target_distance(self,new_distance):
        """ call back function for the OpenCv Slider to set the target distance """
        self.target = new_distance/100.0

    def wall_withslider(self):
        """ Main run loop for wall with slider """ 
        cv2.namedWindow('UI')
        cv2.createTrackbar('distance', 'UI', int(self.target*100), 300, self.set_target_distance)
        r = rospy.Rate(10)
        while not(rospy.is_shutdown()):
            msg = Twist()
            msg.linear.x = self.speed
            msg.angular.z = self.turn
            self.pub.publish(msg)
            cv2.waitKey(10)
            r.sleep()

        
if __name__ == '__main__':
    follower = WallFollower()
    follower.wall_withslider()
