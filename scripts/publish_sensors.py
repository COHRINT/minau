#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from cohrint_minau.msg import Measurement, Positioning
from decimal import Decimal
"""
Publishes
- depth measurements for current robot
- range, bearing, and uncertainty measurements for other active robots
- relative positioning to other robots

range: res 0.1m
azimuth: 0-360 (res 0.1 deg)
elevation: -90 to 90 (res 0.1 deg)
fit error: values toward 0 are more confident, values 2-3 poor fits (res 0.01)
"""

class SensorPub:

    auvs = {}
    
    def __init__(self, name, active_robots):
        self.name = name
        for rob in active_robots:
            rospy.Subscriber('/' + rob + '/pose_gt', Odometry, self.pose_callback)
            self.auvs[rob] = None
        self.pose = None
        rospy.loginfo("Sensor pub for " + name + " initialized")
        self.depth_pub = rospy.Publisher('depth', Float64, queue_size=10)
        depth_rate = 1 / float(rospy.get_param('/sensors/depth_pub_rate'))
        self.depth_timer = rospy.Timer(rospy.Duration(depth_rate), self.depth_callback)
        self.depth_noise = float(rospy.get_param('/sensors/depth_res'))
        self.usbl_pub = rospy.Publisher('usbl', Measurement, queue_size=10)
        self.usbl_pos_pub = rospy.Publisher('usbl_pos', Positioning, queue_size=10)
        usbl_rate = 1 / float(rospy.get_param('/sensors/usbl_pub_rate'))
        self.usbl_timer = rospy.Timer(rospy.Duration(usbl_rate), self.usbl_callback)
        depth_res = Decimal(rospy.get_param('/sensors/depth_res'))
        self.depth_res = - depth_res.as_tuple().exponent # num decimal places
        

    def pose_callback(self, msg):
        topic = msg._connection_header['topic']
        auv = None
        for auv_name in self.auvs.keys():
            if auv_name in topic:
                auv = auv_name
                break
        if auv == self.name:
            self.pose = msg
        else:
            self.auvs[auv] = msg

    def depth_callback(self, msg):
        depth = self.pose.pose.pose.position.z
        depth_data = Float64()        
        depth_data.data = round(depth, self.depth_res)
        self.depth_pub.publish(depth_data)
        
    def usbl_callback(self, msg):
        # loop through auvs
        # generate distance, heading measurements
        # publish that
        # from those measurements calculate the easting, northing and depth relative to world coordinates
        # publish that
        pass

    def get_distance(self, point1, point2):
        pass

    def get_heading(self, pose1, point2):
        """
        Heading from pose1 to point2
        Round to 0.1 degrees resolution
        """
        pass
        

def main():
    rospy.init_node('sensor_pub', anonymous=True)
    ns = rospy.get_namespace()
    active_auvs = rospy.get_param('/active_auvs')
    for auv in active_auvs:
        if auv in ns:
            name = auv
            break
    sp = SensorPub(name, active_auvs)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
