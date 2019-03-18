#!/usr/bin/env python
"""
Simulates the movements of points in space
"""
import rospy
from __future__ import division
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose
import random

class PointSim:

    def __init__(self):
        rospy.loginfo("Point Sim and Controller Initialized")
        self.load_auvs()
        
        self.update_period = 1 / int(rospy.get_param('sim/update_rate_hz'))
        self.timer = rospy.Timer(rospy.Duration(self.update_period), self.update_callback)

    def load_auvs(self):
        self.auvs = {} # each auv has an odom
        auv_list = rospy.get_param('actors_list')
        for auv in auv_list:
            start_odom = Odometry()
            start_pos = rospy.get_param(auv+'/start_pos')
            if start_pos == "random":
                start_odom.pose.pose.position = self.get_random_point()

    def get_random_point(self):
        # returns a random point
        pass

    def update_callback(self, msg):
        # loop through all actors, calculate new poses & publish
        for auv in self.auvs:
            pass
        
        
    def control_callback(self, msg):
        topic = msg._connection_header['topic']
        auv_name = topic.split('/')[1]
        self.auvs[auv_name].twist.twist.linear = msg.linear
        self.auvs[auv_name].twist.twist.angular = msg.angular

def main():
    rospy.init_node('point_sim_contoller')
    ps = PointSim()
    rospy.spin()

if __name__ == "__main__":
    main()
