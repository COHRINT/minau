#!/usr/bin/env python
from __future__ import division
import rosbag
import os
import numpy as np
import rospy
import sys
import tf
import matplotlib.pyplot as plt

"""
This file analyzes the position data from the reservoir testing. It expects a bags/ folder located in the same directory as the file. Uncomment and comment function calls in the if statement at the bottom of the file to analyze certain aspects of the data.

Synchronoization algorithm, look for:
"# Associate the closest timestamped msg of all other imu sources to the first one"
Keep in mind that the algorithm associates timestamps of columns 2-m to the timestamp of the first column.


Purpose of this script: 
1st: How functional are the modems / are they even functional in the reservoir? Comms wise and USBL wise
1.5: Do orientations of pixhawk + modem agree? If not, how reasonable are the modem IMU measurements? Transform them?
2nd: Determine if we can treat the range + angle measurements as separate measurements
3rd purpose: see how correlated the magic number is with bad range measurements -> Run a correlation algorithm

Deliverables:
X 1 - Success vs not success at certain pitches + rolls
    Pitch : scale -90 to 90
    Roll : -90 to 90
2 - Show a plot of all of the range measurements verse time for each bag -> Are they consistent? What's the std
2 - Show a plot of all of the angle measurements verse time for each bag -> consistent? What's the std
3 - Show a 2D plot of all of the range + angle measurements with the magic number overlayed on the color
3 - How correlated was the measurement with the difference from the mean of the measurements? (may need a window here for moving robots)
"""

"""
Topics available
/bluerov2/fleet_communication/beacon_status   11726 msgs    : minau/BeaconStatus                     
/bluerov2/fleet_communication/localization       67 msgs    : geometry_msgs/PoseWithCovarianceStamped
/bluerov2/fleet_communication/ping_status     12793 msgs    : minau/PingStatus                       
/bluerov2/mavros/global_position/local        12045 msgs    : nav_msgs/Odometry                      
/bluerov2/mavros/imu/data                     11467 msgs    : sensor_msgs/Imu                        
/bluerov2/odometry/dead_reckoning             10363 msgs    : nav_msgs/Odometry                      
/bluerov2/odometry/filtered                   10362 msgs    : nav_msgs/Odometry                      
/bluerov2/uuv_control/control_status           3080 msgs    : minau/ControlStatus
"""

def get_local_dir_bag_names():
    # return ["minau_localization_2019-09-05-19-18-15.bag"] # just one file for now
    # return ["minau_localization_2019-09-05-20-10-52.bag"]
    # return ["minau_localization_2019-09-05-20-24-10.bag"]
    return os.listdir(os.getcwd()+"/bags")

def print_modem_success():
    files = get_local_dir_bag_names()
    total_pings_sent = 0
    total_pings_success = 0    

    for f in files:
        bag = rosbag.Bag("bags/" + f)
        send_cnt = 0
        success_cnt = 0
        for topic, msg, t in bag.read_messages(topics=["/bluerov2/fleet_communication/ping_status"]):
            success_cnt = msg.ping_success_count
            send_cnt = msg.ping_send_count
        print("Total Pings Sent: " + str(send_cnt))
        print("Success: "+str(success_cnt))
        print("---")

        total_pings_sent += send_cnt
        total_pings_success += success_cnt
        
        bag.close()

    print("Total Success: " + str(total_pings_success))
    print("Total Pings Sent: " + str(total_pings_sent))
    print("Consistency: " + str(total_pings_success / total_pings_sent))

def print_orientations():
    """ Prints beacon, robot localization, raw Imu, and mavros pitch orientations synchronously """
    files = get_local_dir_bag_names()

    # topic_nickname : topic
    imu_src_topcis = {"beacon": "/bluerov2/fleet_communication/beacon_status", \
                        "mavros" : "/bluerov2/mavros/global_position/local", \
                        "raw_imu" : "/bluerov2/mavros/imu/data", \
                        "robot_loc" : "/bluerov2/odometry/filtered" }

    # Load and associate data
    for f in files:
        print("Processing: " + f)
        bag = rosbag.Bag("bags/" + f)

        data_matrix = [] # Double list, first index is data source, 2nd is list of msgs

        # Load data
        for key in imu_src_topcis:
            msg_data = []
            for topic, msg, t in bag.read_messages(topics=[imu_src_topcis[key]]):
                msg_data.append(msg)
            data_matrix.append(msg_data)

        # Associate the closest timestamped msg of all other imu sources to the first one
        association_matrix = np.zeros([len(data_matrix[0]), len(data_matrix)], dtype=int)
        previous_row = np.zeros([1, len(data_matrix)])
        print(association_matrix.shape)
        try:
            for row in range(association_matrix.shape[0]):
                association_matrix[row,0] = row # Associate the other msgs with respect to this one
                target_time = rospy.Time(data_matrix[0][row].header.stamp.secs, data_matrix[0][row].header.stamp.nsecs).to_sec()
                for col in range(1, association_matrix.shape[1]):
                    while True: # not pretty this alg
                        last_msg_index_for_col = int(previous_row[0, col])
                        last_time = rospy.Time(data_matrix[col][last_msg_index_for_col].header.stamp.secs, \
                                            data_matrix[col][last_msg_index_for_col].header.stamp.nsecs).to_sec()
                        next_time = rospy.Time(data_matrix[col][last_msg_index_for_col+1].header.stamp.secs, \
                                            data_matrix[col][last_msg_index_for_col+1].header.stamp.nsecs).to_sec()
                        last_diff = abs(target_time - last_time)
                        next_diff = abs(target_time - next_time)
                        if last_diff < next_diff:
                            association_matrix[row,col] = last_msg_index_for_col
                            break
                        else:
                            previous_row[0,col] = last_msg_index_for_col + 1
                            # loop
        except IndexError as e: # we've associated all of the data we can
            pass
        # np.set_printoptions(threshold=sys.maxsize)
        # print(association_matrix)

        # Print the pitch angle in a logical manner
        # Nickname | Nickname | Nickname | Nickname
        #  pitch   |   pitch  |  pitch   |   pitch

        line_str = ""
        for key in imu_src_topcis:
            line_str += key + " | "
        print line_str
        for row in range(association_matrix.shape[0]):
            line_str = ""
            for col in range(association_matrix.shape[1]):
                msg = data_matrix[col][association_matrix[row, col]]
                if type(msg).__name__ == "_nav_msgs__Odometry":
                    ori = msg.pose.pose.orientation
                else:
                    ori = msg.orientation
                quat_list = [ori.x, ori.y, ori.z, ori.w]
                roll_rads, pitch_rads, yaw_rads = tf.transformations.euler_from_quaternion(quat_list)
                pitch_degs = round(pitch_rads * (180/np.pi), 2)
                line_str += str(pitch_degs) + " | "
            print(line_str)
        bag.close()

def plot_modem_success_vs_pitch():
    """ Prints beacon, robot localization, raw Imu, and mavros pitch orientations synchronously """
    files = get_local_dir_bag_names()

    total_pitch_array = []
    total_success_array = []
    # Load and associate data
    for f in files:
        print("Processing: " + f)        
        bag = rosbag.Bag("bags/" + f)

        ping_headers = [] # holds headers of all ping attempts
        error_indices = []
        # loop through beacon statuses
        ping_success = 0
        ping_errors = 0
        for topic, msg, t in bag.read_messages(topics=["/bluerov2/fleet_communication/ping_status"]):
            if msg.ping_success_count > ping_success:
                ping_success += 1
                ping_headers.append(msg)
            elif msg.ping_error_count > ping_errors:
                ping_errors += 1
                ping_headers.append(msg)
                error_indices.append(len(ping_headers) - 1)
        
        # Create data matrix
        data_matrix = [] # pings then beacon_statuses
        data_matrix.append(ping_headers)
        msg_data = []
        for topic, msg, t in bag.read_messages(topics=["/bluerov2/fleet_communication/beacon_status"]):
            msg_data.append(msg)
        data_matrix.append(msg_data)
        
        # Associate the closest timestamped msg of all msg sources to the first one
        association_matrix = np.zeros([len(data_matrix[0]), len(data_matrix)], dtype=int)
        previous_row = np.zeros([1, len(data_matrix)])
        try:
            for row in range(association_matrix.shape[0]):
                association_matrix[row,0] = row # Associate the other msgs with respect to this one
                target_time = rospy.Time(data_matrix[0][row].header.stamp.secs, data_matrix[0][row].header.stamp.nsecs).to_sec()
                for col in range(1, association_matrix.shape[1]):
                    while True: # not pretty this alg
                        last_msg_index_for_col = int(previous_row[0, col])
                        last_time = rospy.Time(data_matrix[col][last_msg_index_for_col].header.stamp.secs, \
                                            data_matrix[col][last_msg_index_for_col].header.stamp.nsecs).to_sec()
                        next_time = rospy.Time(data_matrix[col][last_msg_index_for_col+1].header.stamp.secs, \
                                            data_matrix[col][last_msg_index_for_col+1].header.stamp.nsecs).to_sec()
                        last_diff = abs(target_time - last_time)
                        next_diff = abs(target_time - next_time)
                        if last_diff < next_diff:
                            association_matrix[row,col] = last_msg_index_for_col
                            break
                        else:
                            previous_row[0,col] = last_msg_index_for_col + 1
                            # loop
        except IndexError as e: # we've associated all of the data we can
            pass
        np.set_printoptions(threshold=sys.maxsize)

        # Information from ping status is just whether it was successful or not
        success_array = np.ones(len(ping_headers))
        for error_index in error_indices:
            success_array[error_index] = 0
        # print(success_array)

        # Create Pitch array
        pitch_array = np.zeros(len(ping_headers))
        for msg_index in range(len(ping_headers)):
            msg = data_matrix[1][association_matrix[msg_index,1]]
            ori = msg.orientation
            quat_list = [ori.x, ori.y, ori.z, ori.w]
            roll_rads, pitch_rads, yaw_rads = tf.transformations.euler_from_quaternion(quat_list)
            pitch_degs = round(pitch_rads * (180/np.pi), 2)
            pitch_array[msg_index] = pitch_degs
        # print(pitch_array)

        total_pitch_array.extend(pitch_array)
        total_success_array.extend(success_array)

    # Simple Scatter
    # plt.scatter(total_pitch_array, total_success_array)
    # plt.show()

    # bin_data_for_bar_graph(success_array, pitch_array)
    bins, success_height, error_height = bin_data_for_bar_graph(total_success_array, total_pitch_array)
    plt.bar(bins, success_height, width=5, color='g')
    plt.bar(bins, error_height, width=5, color='r')
    plt.xlabel("Robot's Pitch Angle [deg]")
    plt.ylabel("Number of Pings")
    plt.legend(["Successful Pings", "Erroneous Pings"], loc=2)
    plt.title("Number of Pings vs Robot's Pitch Angle")
    plt.show()

def plot_modem_success_vs_roll():
    """ Prints beacon, robot localization, raw Imu, and mavros pitch orientations synchronously """
    files = get_local_dir_bag_names()

    total_pitch_array = []
    total_success_array = []
    # Load and associate data
    for f in files:
        print("Processing: " + f)        
        bag = rosbag.Bag("bags/" + f)

        ping_headers = [] # holds headers of all ping attempts
        error_indices = []
        # loop through beacon statuses
        ping_success = 0
        ping_errors = 0
        for topic, msg, t in bag.read_messages(topics=["/bluerov2/fleet_communication/ping_status"]):
            if msg.ping_success_count > ping_success:
                ping_success += 1
                ping_headers.append(msg)
            elif msg.ping_error_count > ping_errors:
                ping_errors += 1
                ping_headers.append(msg)
                error_indices.append(len(ping_headers) - 1)
        
        # Create data matrix
        data_matrix = [] # pings then beacon_statuses
        data_matrix.append(ping_headers)
        msg_data = []
        for topic, msg, t in bag.read_messages(topics=["/bluerov2/fleet_communication/beacon_status"]):
            msg_data.append(msg)
        data_matrix.append(msg_data)
        
        # Associate the closest timestamped msg of all msg sources to the first one
        association_matrix = np.zeros([len(data_matrix[0]), len(data_matrix)], dtype=int)
        previous_row = np.zeros([1, len(data_matrix)])
        try:
            for row in range(association_matrix.shape[0]):
                association_matrix[row,0] = row # Associate the other msgs with respect to this one
                target_time = rospy.Time(data_matrix[0][row].header.stamp.secs, data_matrix[0][row].header.stamp.nsecs).to_sec()
                for col in range(1, association_matrix.shape[1]):
                    while True: # not pretty this alg
                        last_msg_index_for_col = int(previous_row[0, col])
                        last_time = rospy.Time(data_matrix[col][last_msg_index_for_col].header.stamp.secs, \
                                            data_matrix[col][last_msg_index_for_col].header.stamp.nsecs).to_sec()
                        next_time = rospy.Time(data_matrix[col][last_msg_index_for_col+1].header.stamp.secs, \
                                            data_matrix[col][last_msg_index_for_col+1].header.stamp.nsecs).to_sec()
                        last_diff = abs(target_time - last_time)
                        next_diff = abs(target_time - next_time)
                        if last_diff < next_diff:
                            association_matrix[row,col] = last_msg_index_for_col
                            break
                        else:
                            previous_row[0,col] = last_msg_index_for_col + 1
                            # loop
        except IndexError as e: # we've associated all of the data we can
            pass
        np.set_printoptions(threshold=sys.maxsize)

        # Information from ping status is just whether it was successful or not
        success_array = np.ones(len(ping_headers))
        for error_index in error_indices:
            success_array[error_index] = 0
        # print(success_array)

        # Create Pitch array
        pitch_array = np.zeros(len(ping_headers))
        for msg_index in range(len(ping_headers)):
            msg = data_matrix[1][association_matrix[msg_index,1]]
            ori = msg.orientation
            quat_list = [ori.x, ori.y, ori.z, ori.w]
            roll_rads, pitch_rads, yaw_rads = tf.transformations.euler_from_quaternion(quat_list)
            pitch_degs = round(roll_rads * (180/np.pi), 2)
            pitch_array[msg_index] = pitch_degs
        # print(pitch_array)

        total_pitch_array.extend(pitch_array)
        total_success_array.extend(success_array)

    # Simple Scatter
    # plt.scatter(total_pitch_array, total_success_array)
    # plt.show()

    # bin_data_for_bar_graph(success_array, pitch_array)
    bins, success_height, error_height = bin_data_for_bar_graph(total_success_array, total_pitch_array)
    plt.bar(bins, success_height, width=5, color='g')
    plt.bar(bins, error_height, width=5, color='r')
    plt.xlabel("Robot's Roll Angle [deg]")
    plt.ylabel("Number of Pings")
    plt.legend(["Successful Pings", "Erroneous Pings"], loc=2)
    plt.title("Number of Pings vs Robot's Roll Angle")
    plt.show()
        
def bin_data_for_bar_graph(success_array, orientation_array):
    bins = []
    success_height = []
    error_height = []

    # Loop through all degrees
    for degree in range(60, -60, -10):
        bins.append(degree)
        success_height.append(0)
        error_height.append(0)
        for pitch_index in range(len(orientation_array)):
            if orientation_array[pitch_index] > degree:
                if success_array[pitch_index]: # Was a success
                    success_height[-1] += 1
                else:
                    error_height[-1] += 1
                orientation_array[pitch_index] = -20000 # Make so we won't double count
    print(bins)
    print(success_height)
    print(error_height)
    return bins, success_height, error_height

    # Recursively bin data greater than 60, then greater than 55, then greater than 50 ... 
    # Replace with -200

def range_data_vs_time():
    files = get_local_dir_bag_names()

    all_ranges = []
    all_range_indices = []
    range_avgs = []
    range_stds = []
    count = 0
    # Load and associate data
    for f in files:
        print("Processing: " + f)        
        bag = rosbag.Bag("bags/" + f)
        # Basically load all of the beacon ping data
        ranges = []
        for topic, msg, t in bag.read_messages(topics=["/bluerov2/fleet_communication/localization"]):
            ranges.append(np.linalg.norm([msg.pose.pose.position.x, \
                                        msg.pose.pose.position.y, \
                                        msg.pose.pose.position.z]))

        all_ranges.extend(ranges)
        all_range_indices.extend([count for i in range(len(ranges))])
        # print(all_range_indices)

        range_avgs.append(np.mean(ranges))
        range_stds.append(np.std(ranges))

        count += 1

        # plt.scatter(range(len(ranges)), ranges)
        # plt.show()

        # Plot number of measurements w/ error bars on the std
    # print(range_avgs)
    # print(range_stds)
    # plt.scatter(range_avgs, range_stds)
    # plt.show()
    plt.scatter(all_range_indices, all_ranges)
    plt.xlabel("Experiment Number")
    plt.ylabel("Range Measurement [m]")
    plt.title("Range Measurement vs Experiment Number")
    plt.show()

"""
Goal just plot my favorite bag xy and list the std
If I can also plot a histogram of the range data and list the std

X Step 1: pick my favorite bag
X Step 2: plot the xy range data
X Step 3: Make 2 graphs
X Step 4: Make a histogram & plot range data using my algo to calculate range data
Step 5: Plot range data & variance data on same scale!
Step 6: Print variance of physical distance from the mean of the measurements vs range
"""
def plot_xy():
    files = get_local_dir_bag_names()

    # Load and associate data
    for f in files:
        print("Processing: " + f)        
        bag = rosbag.Bag("bags/" + f)
        # Basically load all of the beacon ping data
        x_vec = []
        y_vec = []
        ranges = []
        for topic, msg, t in bag.read_messages(topics=["/bluerov2/fleet_communication/localization"]):
            x_vec.append(msg.pose.pose.position.x)
            y_vec.append(msg.pose.pose.position.y)
            ranges.append(np.linalg.norm([msg.pose.pose.position.x, \
                                        msg.pose.pose.position.y, \
                                        msg.pose.pose.position.z]))

        plt.figure(figsize=(12, 5))
        plt.subplot(121)
        colors = ["blue" for i in range(len(x_vec))]
        
        print("Number of meas: " + str(len(x_vec)))
        x_std_str = str(round(np.std(x_vec), 2))
        y_std_str = str(round(np.std(y_vec), 2))
        print("Std X: " + x_std_str)
        print("Std Y: " + y_std_str)
        # min_x = int(min(x_vec))
        # max_x = int(max(x_vec))
        # bin_seq = np.linspace(min_x, max_x, num=10)
        # print(bin_seq)
        
        x_vec.append(np.mean(x_vec))
        y_vec.append(np.mean(y_vec))
        colors.append("red")
        x_vec.append(0)
        y_vec.append(0)
        colors.append("green")
        plt.scatter(x_vec, y_vec, color=colors)
        plt.xlabel("y distance [m]")
        plt.ylabel("x distance [m]")
        plt.title("Positional Meas (" + str(len(x_vec)) + ") | X_std: " + x_std_str + " Y_std: " + y_std_str)

        plt.subplot(122)
        plt.hist(ranges, bins=10)
        r_std_str = str(round(np.std(ranges), 2))
        print("Std Range: " + r_std_str)
        plt.xlabel("ranges [m]")
        plt.ylabel("Number of Occurences")
        plt.title("Range Measurements (" + str(len(x_vec)) + ") | R_std: " +  r_std_str)
        plt.show()


if __name__ == "__main__":
    # print_modem_success()
    # plot_modem_success_vs_pitch()
    # print_orientations()
    # plot_modem_success_vs_pitch()
    # plot_modem_success_vs_roll()
    # range_data_vs_time()
    plot_xy()
