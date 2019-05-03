#!/usr/bin/env python

"""
ROS wrapper for the Agent class. Manages data messages from sensor topics, 
and messages to and from other agents.
"""

import os
import yaml
import rospy
import Queue as queue
import numpy as np

from cohrint_minau.msg import depthMeasurement, usblMeasurement, AgentMeasurement, AgentState

class AgentWrapper(object):

    def __init__(self,config_path=None):
        
        # load config file
        # cfg = self.load_config(config_path)

        # initialize ros nose
        rospy.init_node('agent')

        # instantiate Agent object
        # ground_truth = sim.init_ground_truth()
        # self.agent = sim.init_agents(ground_truth,dynamics_fxn,sensors,num_agents=1)

        # set update rate
        # self.update_rate = cfg['update_rate']
        self.update_rate = rospy.get_param('agent_update_rate')
        rate = rospy.Rate(self.update_rate)

        # create local and received meaurement queues
        self.local_measurement_queue = queue.LifoQueue()
        self.local_measurement_cnt = 0
        self.received_measurement_queue = queue.LifoQueue()
        self.received_measurement_cnt = 0

        # create local and recevied covariance intersection queue
        self.local_ci_queue = queue.LifoQueue()
        self.recevied_ci_queue = queue.LifoQueue()

        # create subscribers to sensors
        rospy.Subscriber('depth', depthMeasurement, self.queue_local_measurement)
        rospy.Subscriber('usbl', usblMeasurement, self.queue_local_measurement)

        # create subscribers to comms module
        rospy.Subscriber('agent_meas_incoming', AgentMeasurement, self.queue_received_measurement)
        # rospy.Subscriber('agent_state', AgentState, self.queue_received_state)

        # create publishers to comms module
        self.comms_meas_pub = rospy.Publisher('agent_meas_outgoing', AgentMeasurement, queue_size=10)
        self.comms_state_pub = rospy.Publisher('agent_state_outgoing', AgentState, queue_size=10)

        # begin update loop
        while not rospy.is_shutdown():

            # process all queue measurements and ci messages
            self.update()

            # sleep until next update
            rate.sleep()

    def queue_local_measurement(self,msg):
        """
        Add local measurment to queue, and add to count of local messages.
        """
        self.local_measurement_queue.put(msg)
        self.local_measurement_cnt += 1

    def queue_received_measurement(self,msg):
        """
        Add received measurment to queue.
        """
        self.received_measurement_queue.put(msg)
        self.received_measurement_cnt += 1

    def process_local_measurement_queue(self):
        """
        Empty queue and process measurements.
        """
        # get current number of messages in queue
        num_measurements = self.local_measurement_queue.qsize()
        
        # grab the above number of messages in queue
        local_measurements = [self.local_measurement_queue.get() for x in range(num_measurements)]
        
        return local_measurements

    def process_received_measurement_queue(self):
        """
        Empty queue and process measurements.
        """
        # get current number of messages in queue
        num_measurements = self.received_measurement_queue.qsize()
        
        # grab the above number of messages in queue
        received_measurements = [self.received_measurement_queue.get() for x in range(num_measurements)]
        
        return received_measurements

    def local_ci(self):
        """
        Check if covariance intersection needs to happen. If so, generate service requests.
        """
        pass

    def gen_measurement_msg(self,msg):
        """
        Generate generic between-agent measurement message from local measurement
        message.

        Inputs:

            msg -- local measurement message

        Returns:

            new_msg -- generated message
        """
        new_msg = AgentMeasurement()

        new_msg.type = msg._type.split('/')[1]

        new_msg.header.stamp = rospy.Time.now()

        if msg._type == 'cohrint_minau/usblMeasurement':
            new_msg.data = [msg.range, msg.azimuth, msg.elevation]
        elif msg._type == 'cohrint_minau/depthMeasurement':
            new_msg.data = [msg.data]

        new_msg.status = [1 for x in new_msg.data]

        return new_msg


    def update(self):
        """
        Main update function to process measurements, and ci messages.
        """
        # process measurement queues
        local_measurements = self.process_local_measurement_queue()
        received_measurements = self.process_received_measurement_queue()

        # pass measurements to wrapped agent
        # threshold_msgs = self.agent.process_local_measurements(local_measurements)

        # published thresholded measurements to connections through comms module
        if len(local_measurements) > 0:
            msg = self.gen_measurement_msg(local_measurements[0])
            self.comms_meas_pub.publish(msg)

        # self.agent.process_received_measurements(received_measurements)



if __name__ == "__main__":
    AgentWrapper()