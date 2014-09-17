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

class ObstacleAvoid():
    def __init__(self, target=1, buf_big=.2, buf_small=.05):
        rospy.init_node('avoid_obstacle', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)

        self.turn = 0.0
        self.speed = 0.0

    def switch_state(self,state):
        self.state=state
        self.pub.publish(Twist())

    def angle(self,data,i):
        return data.angle_min+(data.angle_increment*i)

    def goodness(self,r,angle):
        if r>3:
            return (abs(angle-3.1416)/3.1416)
        else:
            return (abs(angle-3.1416)/3.1416)*(r-.2)/2.8

    def scan_received(self,data):
        angles = [data.angle_min+(data.angle_increment*i) for i in range(len(data.ranges))]
        vals = [(self.goodness(r,a),a) for r,a in zip(data.ranges,angles) if r>0]
        goal = max(vals)[1]

        if goal<3.1416:
            self.turn = 2.5*goal/3.1416
        else:
            self.turn = 2.5*(goal-3.14156)/3.14156
        self.speed = .1*(data.ranges[0]-.15)

        print "ahead", vals[0]
        print "best", max(vals)
        print "goal", goal
        print "turn",self.turn
        print "speed",self.speed

    def wall_withslider(self):
        """ Main run loop for wall with slider """ 
        r = rospy.Rate(10)
        while not(rospy.is_shutdown()):
            msg = Twist()
            msg.linear.x = self.speed
            msg.angular.z = self.turn
            self.pub.publish(msg)
            r.sleep()

        
if __name__ == '__main__':
    follower = ObstacleAvoid()
    follower.wall_withslider()
