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
from sensor_msgs.msg import PointCloud2, Image

#from human_position.msg import Object

#from human_position.srv import SetDistanceOpenpose
from geometry_msgs.msg import PointStamped

#from erasers_hmi_ros.srv import ShowImageDisplay, ShowImageDisplayRequest

import rospkg
rospack = rospkg.RosPack()
#from common import speech
#from common.smach_states.utils import TemporarySubscriber

class DetectPerson(smach.State):
    def __init__(self, robot=None,
                 split_openpose = False,
                 split_openpose_plan = None,
                 max_depth=4.0,  
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
        self.max_depth = max_depth
        self.split_openpose = split_openpose
        self.split_openpose_plan = split_openpose_plan

        from erasers_hmi_ros.srv import ShowImageDisplay, ShowImageDisplayRequest
        rospy.loginfo("waiting for show image display node (erasers_hmi_ros)")
        rospy.wait_for_service('show_image_to_display')
        self.show_image = rospy.ServiceProxy('show_image_to_display', ShowImageDisplay)

    def callback(self, data):
        #print("cb")
        if self.human_coordinate is None:
            print("Waiting for receiving 3d data")
            self.found = True
            self.human_coordinate = data
            print(self.human_coordinate)
            # time

    #def img_callback(self, data):
    #    #show display image service client
    #    from erasers_hmi_ros.srv import ShowImageDisplay, ShowImageDisplayRequest
    #    try:
    #        msg = ShowImageDisplayRequest()
    #        msg.image = data
    #        msg.time = 10
    #        self.show_image(msg)

    #    except rospy.ServiceException as e:
    #        rospy.logerr("Service call failed: %s" % e)

    def execute(self, userdata):
        try:
            rospy.logwarn('##### split #####')
            try:
                if self.split_openpose:
                    self.found = False
                    self.human_coordinate = None
                    start_time = rospy.Time.now()
                    if self.split_openpose_plan == 'execute':
                        set_disp = "xhost " + '+'
                        start_op = Popen(set_disp, shell=True)
                        cmd = "exec " + 'docker compose -f ' + rospack.get_path('hsr_gui') + '/../openpose-ros-docker/handsup-docker-compose.yml up'
                        processes = Popen(cmd, shell=True)
                        print ("op_pub is excuted")
            
                        #this service for set custom distance value
                        #rospy.wait_for_service('set_distance')
                        #try:
                        #    set_dist = rospy.ServiceProxy('set_distance',SetDistanceOpenpose)
                        #    set_dist(self.max_depth)
                        #    print("current distance =",self.max_depth)
                        #except rospy.ServiceException as e:
                        #    rospy.logerr("Service call failed: %s" % e)
                        return 'success'
                    
                    elif self.split_openpose_plan == 'hands_up':
                        sub = rospy.Subscriber('op_result', PointStamped, self.callback) #TODO service
                        sub_bbox = rospy.Subscriber('detected_human_image', Image, self.img_callback) #TODO service
                        while not rospy.is_shutdown():
                            if self.found:
                                userdata.human_coordinate = self.human_coordinate
                                rospy.logwarn('##### coordinate #####')
                                sub.unregister()
                                sub_bbox.unregister()
                                return 'success'
                        
                            if (rospy.Time.now() - start_time).to_sec() > self.timeout:  #timeout error
                                return 'timeout'

                    elif self.split_openpose_plan == 'kill_openpose':
                        cmd_down= "exec " + 'docker compose -f ' + rospack.get_path('hsr_gui') + '/../openpose-ros-docker/handsup-docker-compose.yml down'
                        kill = Popen(cmd_down, shell=True)
                        return 'success'
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

            rospy.logerr('##### Detect person #####')
            self.found = False
            self.human_coordinate = None
            start_time = rospy.Time.now()

            set_disp = "xhost " + '+'
            start_op = Popen(set_disp, shell=True)
            cmd = "exec " + 'docker compose -f ' + rospack.get_path('hsr_gui') + '/../openpose-ros-docker/handsup-docker-compose.yml up'
            processes = Popen(cmd, shell=True)
            print ("op_pub is excuted")
            
            #this service for set custom distance value
            #rospy.wait_for_service('set_distance')
            #try:
            #    set_dist = rospy.ServiceProxy('set_distance',SetDistanceOpenpose)
            #    set_dist(self.max_depth)
            #    print("current distance =",self.max_depth)
            #except rospy.ServiceException as e:
            #    rospy.logerr("Service call failed: %s" % e)
            
            sub = rospy.Subscriber('op_result', PointStamped, self.callback) #TODO service
            sub_bbox = rospy.Subscriber('detected_human_image', Image, self.img_callback) #TODO service

            while not rospy.is_shutdown():
                if self.found:
                    userdata.human_coordinate = self.human_coordinate
                    cmd_down= "exec " + 'docker compose -f ' + rospack.get_path('hsr_gui') + '/../openpose-ros-docker/handsup-docker-compose.yml down'
                    kill = Popen(cmd_down, shell=True)
                    sub.unregister()
                    sub_bbox.unregister()
                    return 'success'


                if (rospy.Time.now() - start_time).to_sec() > self.timeout:  #timeout error
                    cmd_down = "exec " + 'docker compose -f ' + rospack.get_path('hsr_gui') + '/../openpose-ros-docker/handsup-docker-compose.yml down'
                    kill = Popen(cmd_down, shell=True)
                    ub.unregister()
                    #sub_bbox.unregister()
                    return 'timeout'
                
        except:
            rospy.logerr(traceback.format_exc())
            cmd_down = "exec " + 'docker compose -f ' + rospack.get_path('hsr_gui') + '/../openpose-ros-docker/handsup-docker-compose.yml down'
            kill = Popen(cmd_down, shell=True)
            #TODO
            #sub.unregister()
            #sub_bbox.unregister()
            return 'failure'

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
                                                                    
