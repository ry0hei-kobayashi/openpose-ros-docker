#!/usr/bin/env python3
# coding:UTF-8

import os
import subprocess
from subprocess import Popen
import math
import smach
import time
import cv2
import rospy
from cv_bridge import CvBridge
import ros_numpy
import numpy as np
import collections
import traceback
from sensor_msgs.msg import PointCloud2
#from depth_lib.msg import Object
from human_position.msg import Object
from geometry_msgs.msg import PointStamped

import rospkg
rospack = rospkg.RosPack()
#from common import speech
#from common.smach_states.utils import TemporarySubscriber

class DetectPerson_execute(smach.State):
    def __init__(self, robot=None,
                 max_depth = 4.0,
                 timeout=30.,
                 say_fn=None,
                 prompt_msg="detecting_person.",
                 success_msg="OK."):
        smach.State.__init__(self,
                             outcomes=['success', 'failure'],
                             output_keys=['human_coordinate'])
        self.found = False
        self.timeout = timeout
        #self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.prompt_msg = prompt_msg
        self.success_msg = success_msg
        self.human_coordinate = None

    def callback(self, data):
        print("cb")
        if self.human_coordinate is None:
            print("Waiting for receiving 3d data")
            self.found = True
            self.human_coordinate = data
            print(self.human_coordinate)
            # time

    def execute(self, userdata):
        try:
            self.found = False
            self.human_coordinate = None
            start_time = rospy.Time.now()

            set_disp = "xhost " + '+'
            start_op = Popen(set_disp, shell=True)

            cmd = "exec " + 'docker compose -f ' + rospack.get_path('hsr_gui') + '/../openpose-ros-docker/handsup-docker-compose.yml up'
            processes = Popen(cmd, shell=True)
            print ("op_pub is excuted")
            sub = rospy.Subscriber('op_result', PointStamped, self.callback)
            return 'success'
            
        except:
            rospy.logerr(traceback.format_exc())
            sub.unregister()
            return 'failure'

class DetectPerson_get_point(smach.State):
    def __init__(self, robot=None,
                 timeout=30.,
                 say_fn=None,
                 prompt_msg="detecting_person.",
                 success_msg="OK."):
        smach.State.__init__(self,
                             outcomes=['success', 'failure', 'timeout'],
                             output_keys=['human_coordinate'])
        self.found = False
        self.timeout = timeout
        #self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.prompt_msg = prompt_msg
        self.success_msg = success_msg
        self.human_coordinate = None
        self.detected_human_image = None

    def callback(self, data):
        print("cb")
        if self.human_coordinate is None:
            print("Waiting for receiving 3d data")
            self.found = True
            self.human_coordinate = data
            print(self.human_coordinate)
            # time

    def img_callback(self, data):
        print("imgcb")
        if self.detected_human_image is None:
            self.found = True
            self.detected_human_image = data
            #print(self.detected_human_image)

    def execute(self, userdata):
        try:
            self.found = False
            self.human_coordinate = None
            start_time = rospy.Time.now()

            set_disp = "xhost " + '+'
            start_op = Popen(set_disp, shell=True)

            cmd = "exec " + 'docker compose -f ' + rospack.get_path('hsr_gui') + '/../openpose-ros-docker/handsup-docker-compose.yml up'
            processes = Popen(cmd, shell=True)
            print ("op_pub is excuted")

            #this service for set custom distance value
            rospy.wait_for_service('set_distance')
            set_dist = rospy.ServiceProxy('set_distance',SetDistanceOpenpose)
            set_dist(self.max_depth)
            print("current distance =",self.max_depth)

            sub = rospy.Subscriber('op_result', PointStamped, self.callback)
            sub_bbox = rospy.Subscriber('detected_human_image', Image, self.img_callback) #TODO service

            #with TemporarySubscriber('op_result', Object, self.callback):
            # print("sub")
            while not rospy.is_shutdown():
                if self.found:
                    userdata.human_coordinate = self.human_coordinate
                    userdata.detected_human_image = self.detected_human_image
                    sub.unregister()
                    return 'success'
                if (rospy.Time.now() - start_time).to_sec() > self.timeout:  #timeout error
                    sub.unregister()
                    return 'timeout'

        except:
            rospy.logerr(traceback.format_exc())
            sub.unregister()
            return 'failure'

class DetectPerson_kill(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'])

    def execute(self, userdata):
        cmd_down = "exec " + 'docker compose -f ' + rospack.get_path('hsr_gui') + '/../openpose-ros-docker/handsup-docker-compose.yml down'
        kill = Popen(cmd_down, shell=True)
        return 'success'
        
if __name__ == "__main__":
    rospy.init_node('smach_example_state_machine')

    sm = smach.StateMachine(outcomes=['success', 'failure'])
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('DEBUG', DetectPerson(),
                               transitions={'success': 'success',
                                            'timeout': 'failure',
                                            'failure': 'failure'})

    sm.execute()
                                                                    
