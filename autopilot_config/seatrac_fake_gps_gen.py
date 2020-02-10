#!/usr/bin/env python

from __future__ import division

"""
Fake gps generation node.
"""

import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HilGPS

class SeaTracFakeGPS:

    def __init__(self):

        rospy.init_node('fake_gps_gen')

        self.fake_gps_pub = rospy.Publisher('/mavros/fake_gps/mocap/pose',PoseStamped,queue_size=10)
        #self.mocap_pub = rospy.Publisher('/mavros/mocap/pose',PoseStamped,queue_size=10)
        #self.fake_hil_gps_pub = rospy.Publisher('/bluerov2/mavros/hil/gps',HilGPS,queue_size=10)

        self.current_pose = np.array([0.0,0.0,0.0])

        pub_rate = 10 # Hz
        self.pub_timer = rospy.Rate(pub_rate)

        while not rospy.is_shutdown():

            self.current_pose += 0.1

            #self.pub_mocap_data(self.current_pose)
            self.pub_fake_gps_data(self.current_pose)
            # self.pub_hil_gps(self.current_pose)

            self.pub_timer.sleep()

    def pub_mocap_data(self,current_pose):

        # create message
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = current_pose

        self.mocap_pub.publish(msg)

    def pub_fake_gps_data(self,current_pose):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = current_pose

        self.fake_gps_pub.publish(msg)

    def pub_hil_gps(self, current_pose):

        lla = self.cart2LLA(*current_pose)

        # create msg
        msg = HilGPS()
        msg.fix_type = 3
        msg.geo.longitude = lla[0]
        msg.geo.latitude = lla[1]
        msg.geo.altitude = current_pose[2]/1000

        msg.eph = 2.0
        msg.epv = 2.0
        msg.vel = 65535
        msg.vn = 0
        msg.ve = 0
        msg.vd = 0
        msg.cog = 65535
        msg.satellites_visible = 255

        self.fake_hil_gps_pub.publish(msg)

    def cart2LLA(self,x,y,z):
        pi = 3.14159265359
        alpha = pi / 2
        r = 6371000000
        lat0 = (39.894470 * pi) / 180
        long0 = (32.777973 * pi) / 180

        dlat = (x * math.cos(alpha) + y * math.sin(alpha)) / r
        dlong = (x * math.sin(alpha) - y * math.cos(alpha)) / r

        lla = [((long0 + dlong) * 180) / pi, ((lat0 + dlat) * 180) / pi]

        return lla



if __name__ == "__main__":
    SeaTracFakeGPS()

