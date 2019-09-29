#!/usr/bin/env python
from __future__ import division
from minau.msg import ControlStatus
from minau.srv import ArmControl, DisarmControl, SetHeadingDepth, SetHeadingVelocity
import rospy
import numpy as np
from geometry_msgs.msg import Vector3

def set_head_depth(heading, depth, setpoint_thresh):
    rospy.loginfo("Setting head/depth")
    reached_depth = False
    while not rospy.is_shutdown():
        head_depth = rospy.ServiceProxy(ns + "uuv_control/set_heading_depth", SetHeadingDepth)
        try:
            resp = head_depth(heading, abs(depth))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        status = rospy.wait_for_message("uuv_control/control_status", ControlStatus)
        position = status.current_position
        if abs(depth - position.z) < setpoint_thresh:
            break
        rospy.sleep(2)

def waypoint(point_list, max_vel, setpoint_thresh):
    status = rospy.wait_for_message("uuv_control/control_status", ControlStatus)
    position = status.current_position

    xdiff = point_list[0] - position.x
    ydiff = point_list[1] - position.y
    heading = np.arctan2(ydiff, xdiff) * (180 / np.pi)
    set_head_depth(heading, point_list[2], setpoint_thresh)

    distance = np.linalg.norm([xdiff, ydiff])
    vel = Vector3()
    head_vel = rospy.ServiceProxy(ns + "uuv_control/set_heading_velocity", SetHeadingVelocity)
    heading_rads = 0.0
    rospy.loginfo("Setting Heading Velocity")
    while not rospy.is_shutdown() and distance > setpoint_thresh:
        status = rospy.wait_for_message("uuv_control/control_status", ControlStatus)
        position = status.current_position
        xdiff = point_list[0] - position.x
        ydiff = point_list[1] - position.y
        distance = np.linalg.norm([xdiff, ydiff])
        heading_rads = np.arctan2(ydiff, xdiff)
        vel.x = max_vel * np.cos(heading_rads)
        vel.y = max_vel * np.sin(heading_rads)

        # Slowdown as we approach
        vel.x = vel.x if distance < 3 else vel.x * (distance / 4)
        vel.y = vel.y if distance < 3 else vel.y * (distance / 4)

        try:
            resp = head_vel(heading_rads * (180/np.pi), vel)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        rospy.sleep(1)

    rospy.loginfo("Setpoint reached.")

    # Stop the sub
    resp = False
    vel = Vector3(); vel.x = 0.0; vel.y = 0.0; vel.z = 0.0
    while not rospy.is_shutdown() and not resp:
        try:            
            resp = head_vel(heading_rads * (180/np.pi), vel)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
    
    
rospy.init_node("planner_node")
ns = rospy.get_namespace()

rospy.logwarn("Waiting for service: " + ns + "/uuv_control/arm_control")
rospy.wait_for_service(ns + "uuv_control/arm_control")
rospy.logwarn("Found Service " + ns + "uuv_control/arm_control")

armed = False
while not rospy.is_shutdown() and not armed:
    arm = rospy.ServiceProxy(ns + "uuv_control/arm_control", ArmControl)
    try:
        resp = arm()
        rospy.sleep(2)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    status = rospy.wait_for_message("uuv_control/control_status", ControlStatus)
    armed = status.armed

# loop through actions
if not rospy.has_param('planner/action0/point'):
    raise("No actions found in planner for " + ns + " !!")

action_num = 0
action_available = True
max_vel = rospy.get_param("planner/configurations/max_vel")
setpoint_thresh = rospy.get_param("planner/configurations/setpoint_thresh")

while not rospy.is_shutdown() and action_available:
    point_list = rospy.get_param("planner/action" + str(action_num) + "/point")

    waypoint(point_list, max_vel, setpoint_thresh)
    if rospy.has_param("planner/action" + str(action_num) + "/repeat"):
        action_num = rospy.get_param("planner/action" + str(action_num) + "/repeat")
    else:
        action_num += 1
    action_available = rospy.has_param("planner/action" + str(action_num) + "/point")

# Disarm bluerov
armed = True
while not rospy.is_shutdown() and armed:
    disarm = rospy.ServiceProxy(ns + "uuv_control/disarm_control", DisarmControl)
    try:
        resp = disarm()
        rospy.sleep(2)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    status = rospy.wait_for_message("uuv_control/control_status", ControlStatus)
    armed = status.armed

rospy.loginfo("No more actions available")