#!/usr/bin/env python
from __future__ import division

"""
STILL IN DEVELOPMENT, NOT USABLE YETx
"""
import rospy
import random
from geometry_msgs.msg import TwistStamped, Vector3, PoseStamped
import numpy as np
from minau.srv import ArmControl, SetHeadingVelocity
from nav_msgs.msg import Odometry
import tf

class Planner:

    def __init__(self, name, update_period):
        self.name = name
        self.pub = rospy.Publisher('new_twist', TwistStamped, queue_size=10)
        self.planner = rospy.get_param('planner')
        start_twist = rospy.get_param('start_twist')
        if start_twist == 'random':
            self.twist = self.get_random_twist()
        else:
            self.twist = self.load_twist_dict(start_twist)
        rospy.Subscriber('/' + name + '/pose_gt', Odometry, self.pose_callback)
        self.listener = tf.TransformListener()
        rospy.wait_for_message('/' + name + '/pose_gt', Odometry)
        rospy.Timer(rospy.Duration(update_period), self.update_auv_callback)
        rospy.loginfo(name + " Planner Initialized.")
        self.seq = 0
        if self.planner != 'stopped':
            self.arm_uav()
        else:
            rospy.logwarn("Not arming: " + name)

        # Probably make a first timer to help us navigate to the starting position & then begin the 2nd timer

    def pose_callback(self, msg):
        self.pose = msg.pose.pose

    def arm_uav(self):
        rospy.loginfo("Arming " + self.name)
        srv_name = '/' + self.name + '/uuv_control/arm_control'
        rospy.wait_for_service(srv_name)
        req = rospy.ServiceProxy(srv_name, ArmControl)
        try:
            res = req()
            # rospy.logfatal(res)
        except rospy.ServiceException as exc:
            print("No response to " + srv_name)
        
    def load_twist_dict(self, twist_dict):
        dot_x = float(twist_dict['x'])
        dot_y = float(twist_dict['y'])
        dot_z = float(twist_dict['z'])
        dot_psi = float(twist_dict['psi'])
        twist = TwistStamped()
        twist.twist.linear.x = dot_x
        twist.twist.linear.y = dot_y
        twist.twist.linear.z = dot_z
        twist.twist.angular.z = dot_psi
        return twist

    def get_random_twist(self):
        [min_twist, max_twist] = rospy.get_param('/planners/random_linear_vel_range')
        [min_ang, max_ang] = rospy.get_param('/planners/random_angular_vel_range')
        size = max_twist - min_twist
        twist = TwistStamped()
        twist.twist.linear.x = random.random() * size + min_twist
        twist.twist.linear.y = random.random() * size + min_twist
        twist.twist.linear.z = random.random() * size + min_twist
        size_ang = max_ang - min_ang
        twist.twist.angular.z = random.random() * size_ang + min_ang
        return twist

    def set_hv(self, heading, velocity):
        """ Sends a BlueROV along the desired velocity
        
        Parameters
        ----------
        heading : float
        velocity : geometry_msgs/Vector3

        Returns
        -------
        None
        """
        srv_name = '/' + self.name + '/uuv_control/set_heading_velocity'
        rospy.wait_for_service(srv_name)
        req = rospy.ServiceProxy(srv_name, SetHeadingVelocity)
        try:
            req(heading, velocity)
        except rospy.ServiceException as exc:
            print("No response to " + srv_name)

    def update_auv_callback(self, msg):
        if self.planner == "stopped": # Nothing to do if we're stopped
            return

        # Need to transform self.twist.twist.linear to world ned frame
        pose = PoseStamped()
        pose.pose.position.x = self.twist.twist.linear.x - self.pose.position.x
        pose.pose.position.y = self.twist.twist.linear.y - self.pose.position.y
        pose.pose.position.z = self.twist.twist.linear.z - self.pose.position.z
        pose.pose.orientation = self.pose.orientation
        pose.header.frame_id = self.name + '/base_link'
        new_pose = self.listener.transformPose('/world_ned', pose)
        rospy.loginfo("sub pose: " + str(self.pose))
        rospy.loginfo("new pose: " + str(new_pose))
        return
    
        v = Vector3()
        v.x = new_pose.pose.position.x
        v.y = new_pose.pose.position.y
        v.z = new_pose.pose.position.z
        
        quat_list = [ self.pose.orientation.x, \
                      self.pose.orientation.y, \
                      self.pose.orientation.z, \
                      self.pose.orientation.w ]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quat_list)
        yaw = (yaw * 180) / ( 2 * np.pi)
        rospy.loginfo("yaw: " + str(yaw))
        heading = yaw # + self.twist.twist.angular.z
        self.set_hv(heading, v)
        rospy.loginfo("Updating AUV")
        rospy.loginfo("New yaw: " + str(yaw))
        
    def pub_cmd(self, msg):
        new_twist = self.get_new_twist()
        if new_twist == None: # publish the old velocity
            new_twist = self.twist
        else:
            self.twist = new_twist
        new_twist.header.seq = self.seq
        new_twist.header.stamp = rospy.Time.now()
        new_twist.header.frame_id = self.name + "/base_link"
        self.pub.publish(new_twist)
        self.seq += 1

    def get_new_twist(self):
        """ This function provides an easy place to add more complex planners  that actively change the velocity """
        if self.planner == "linear":
            return None

def main():
    rospy.init_node('point_planner', anonymous=True)
    name = rospy.get_namespace().split('/')[1]
    rospy.loginfo("waiting for params to become available")
    while not rospy.has_param('/planners/update_freq') and not rospy.is_shutdown(): # wait for the param server to load
        pass
    rospy.loginfo("params found")
    param_name = rospy.search_param('planners/update_freq')
    update_period = 1 / int(rospy.get_param(param_name))
    p = Planner(name, update_period)
    rospy.spin()
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
