#!/usr/bin/env python3
# coding:UTF-8

######	    ######
## For DSPL     ##
######      ######

import collections
import math
import os
import subprocess
import time
import traceback
from collections import OrderedDict
from subprocess import PIPE, Popen, call
from time import sleep

import cv2
import numpy as np
from numpy import linalg as LA
import psutil
import ros_numpy
import rospy
import smach
import tf2_geometry_msgs
import tf2_ros
import tf
from cv_bridge import CvBridge
#import geometry_msgs.msg
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from human_position.msg import HumanCoordinates
#from common.smach_states.utils import TemporarySubscriber

# HSRB Library 
#import hsrb_interface 
#from hsrb_interface import geometry 
 
# eR@sers OUR Library 
#from common import speech 



NOSE = 0
NECK = 1
R_SHOULD = 2
R_ELBOW = 3
R_HAND = 4
L_SHOULD = 5
L_ELBOW = 6
L_HAND = 7
C_WAIST = 8
R_WAIST = 9
R_NEE = 10
R_FOOT = 11
L_WAIST = 12
L_NEE = 13
L_FOOT = 14
R_EYE = 15
L_EYE = 16
R_EAR = 17
L_EAR = 18

class PointingPerson(smach.State):
    def __init__(self, robot=None,
                 timeout=20.,
                 say_fn=None,
                 tfBuffer=None,
                 prompt_msg="Pointing Detector by RyoheiSoft",
                 xtion_height=1.00,#1.17
                 xtion_max_depth=5.00,
                 xtion_min_depth=0.30,
                 xtion_left=-1.20,
                 xtion_right=1.20,
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

        self.pub = rospy.Publisher("marker_pub", MarkerArray, queue_size=10)

        self.xtion_height = xtion_height
        self.xtion_max_depth = xtion_max_depth
        self.xtion_min_depth = xtion_min_depth
        self.xtion_left = xtion_left
        self.xtion_right = xtion_right

        if tfBuffer is None:
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)

        else:
            self.tfBuffer = tfBuffer
        self.br = tf.TransformBroadcaster()

    def delete_marker(self):
        markerArray = MarkerArray()
        marker = Marker()
        marker.action = marker.DELETEALL
        marker.id = np.random.randint(100)
        markerArray.markers.append(marker)
        self.pub.publish(markerArray)

    def plot_marker(self, camera_hand, rgb=(1,0,0)):

        marker_data = Marker()
        marker_data.header.frame_id = "head_rgbd_sensor_rgb_frame"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.type = Marker.SPHERE
        marker_data.ns = "shapes"
        marker_data.id = np.random.randint(1000000)
        marker_data.action = marker_data.ADD

        marker_data.pose.position.x = camera_hand[0]
        marker_data.pose.position.y = camera_hand[1]
        marker_data.pose.position.z = camera_hand[2]

        marker_data.pose.orientation.x = 0.0
        marker_data.pose.orientation.y = 0.0
        marker_data.pose.orientation.z = 0.0
        marker_data.pose.orientation.w = 1.0

        marker_data.color.r = rgb[0]
        marker_data.color.g = rgb[1]
        marker_data.color.b = rgb[2]
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.1
        marker_data.scale.z = 0.1

        marker_data.lifetime = rospy.Duration()

        return marker_data

    def callback(self, data):
        self.delete_marker()
        if self.human_coordinate is None:
            #print("Waiting for receiving 3d data")
            # self.found = True
            threed = HumanCoordinates()
            threed.header.stamp = rospy.Time.now()
            threed.header.frame_id = "head_rgbd_sensor_rgb_frame"

            threed.r_elbow.position.x = data.poses[R_ELBOW].position.x
            threed.r_elbow.position.y = data.poses[R_ELBOW].position.y
            threed.r_elbow.position.z = data.poses[R_ELBOW].position.z

            threed.r_hand.position.x = data.poses[R_HAND].position.x
            threed.r_hand.position.y = data.poses[R_HAND].position.y
            threed.r_hand.position.z = data.poses[R_HAND].position.z

            threed.r_should.position.x = data.poses[R_SHOULD].position.x
            threed.r_should.position.y = data.poses[R_SHOULD].position.y
            threed.r_should.position.z = data.poses[R_SHOULD].position.z

            threed.l_elbow.position.x = data.poses[L_ELBOW].position.x
            threed.l_elbow.position.y = data.poses[L_ELBOW].position.y
            threed.l_elbow.position.z = data.poses[L_ELBOW].position.z

            threed.l_hand.position.x = data.poses[L_HAND].position.x
            threed.l_hand.position.y = data.poses[L_HAND].position.y
            threed.l_hand.position.z = data.poses[L_HAND].position.z

            threed.l_should.position.x = data.poses[L_SHOULD].position.x
            threed.l_should.position.y = data.poses[L_SHOULD].position.y
            threed.l_should.position.z = data.poses[L_SHOULD].position.z

            threed.neck.position.x = data.poses[NECK].position.x
            threed.neck.position.y = data.poses[NECK].position.y
            threed.neck.position.z = data.poses[NECK].position.z

            threed.c_waist.position.x = data.poses[C_WAIST].position.x
            threed.c_waist.position.y = data.poses[C_WAIST].position.y
            threed.c_waist.position.z = data.poses[C_WAIST].position.z

            r_elbow_hand = np.array([threed.r_hand.position.x - threed.r_elbow.position.x, \
                                     threed.r_hand.position.y - threed.r_elbow.position.y, \
                                     threed.r_hand.position.z - threed.r_elbow.position.z])
 
            r_camera_elbow = np.array([threed.r_hand.position.x - 0, \
                                       threed.r_hand.position.y - 0, \
                                       threed.r_hand.position.z - 0])

            l_elbow_hand = np.array([threed.l_hand.position.x - threed.l_elbow.position.x, \
                                     threed.l_hand.position.y - threed.l_elbow.position.y, \
                                     threed.l_hand.position.z - threed.l_elbow.position.z])

            l_camera_elbow = np.array([threed.l_hand.position.x - 0, \
                                       threed.l_hand.position.y - 0, \
                                       threed.l_hand.position.z - 0])

            r_reference = np.array([threed.r_elbow.position.x - threed.r_elbow.position.x,
                                    threed.r_elbow.position.y - (threed.r_elbow.position.y + 0.5),
                                    threed.r_elbow.position.z - threed.r_elbow.position.z])

            l_reference = np.array([threed.l_elbow.position.x - threed.l_elbow.position.x,
                                    threed.l_elbow.position.y - (threed.l_elbow.position.y + 0.5),
                                    threed.l_elbow.position.z - threed.l_elbow.position.z])


            marker_array = MarkerArray()

            r_pose_array = PoseArray()
            r_pose_array.header.stamp = rospy.Time.now()
            r_pose_array.header.frame_id = "head_rgbd_sensor_rgb_frame"

            l_pose_array = PoseArray()
            l_pose_array.header.stamp = rospy.Time.now()
            l_pose_array.header.frame_id = "head_rgbd_sensor_rgb_frame"

            t = 1
            t1 = 1

            while not rospy.is_shutdown():
                r_camera_hand = r_camera_elbow + t * r_elbow_hand
                r_distance = math.sqrt((r_camera_elbow[0] - r_camera_hand[0])**2 + (r_camera_elbow[1] - r_camera_hand[1])**2+(r_camera_elbow[2] - r_camera_hand[2])**2)
                l_camera_hand = l_camera_elbow + t1 * l_elbow_hand
                l_distance = math.sqrt((l_camera_elbow[0] - l_camera_hand[0])**2 + (l_camera_elbow[1] - l_camera_hand[1])**2+(l_camera_elbow[2] - l_camera_hand[2])**2)
                
                if (r_distance and l_distance) < self.xtion_max_depth:
                    if self.xtion_height > (r_camera_hand[1] and l_camera_hand[1]):

                        marker_data = self.plot_marker(r_camera_hand, rgb=(1,0,0))
                        marker_array.markers.append(marker_data)
                        marker_data = self.plot_marker(l_camera_hand, rgb=(0,1,1))
                        marker_array.markers.append(marker_data)

                        r_pose = Pose()
                        r_pose.position.x = r_camera_hand[0]
                        r_pose.position.y = r_camera_hand[1]
                        r_pose.position.z = r_camera_hand[2]
                        r_pose.orientation.w = 1.0
                        r_pose_array.poses.append(r_pose)

                        l_pose = Pose()
                        l_pose.position.x = l_camera_hand[0]
                        l_pose.position.y = l_camera_hand[1]
                        l_pose.position.z = l_camera_hand[2]
                        l_pose.orientation.w = 1.0
                        l_pose_array.poses.append(l_pose)

                        t += 1
                        t1 += 1

                    else:
                        # print("break_in")
                        break
                else:
                    # print("break")
                    break

            self.pub.publish(marker_array)

            ### xtion frame to base_footprint frame coordinate ###
            r_pose_stamped = PoseStamped()
            r_pose_stamped.header.frame_id = data.header.frame_id
            r_pose_stamped.header.stamp = rospy.Time.now()
            r_pose_stamped.pose.position.x = r_pose_array.poses[-1].position.x
            r_pose_stamped.pose.position.y = r_pose_array.poses[-1].position.y
            r_pose_stamped.pose.position.z = r_pose_array.poses[-1].position.z
            r_pose_stamped.pose.orientation.w = 1
            print(r_pose_stamped)

            l_pose_stamped = PoseStamped()
            l_pose_stamped.header.frame_id = data.header.frame_id
            l_pose_stamped.header.stamp = rospy.Time.now()
            l_pose_stamped.pose.position.x = l_pose_array.poses[-1].position.x
            l_pose_stamped.pose.position.y = l_pose_array.poses[-1].position.y
            l_pose_stamped.pose.position.z = l_pose_array.poses[-1].position.z
            l_pose_stamped.pose.orientation.w = 1
            print(l_pose_stamped)

            c_pose_stamped = PoseStamped()
            c_pose_stamped.header.frame_id = data.header.frame_id
            c_pose_stamped.header.stamp = rospy.Time.now()
            c_pose_stamped.pose.position.x = threed.c_waist.position.x 
            c_pose_stamped.pose.position.y = threed.c_waist.position.y
            c_pose_stamped.pose.position.z = threed.c_waist.position.z 
            c_pose_stamped.pose.orientation.w = 1
            #print(c_pose_stamped)

            try:
                # cam2map=self.tfBuffer.transform(pose_stamped,"odom",timeout=rospy.Duration(1))
                r_pose2map = self.tfBuffer.transform(r_pose_stamped, 'map', timeout=rospy.Duration(1))
                l_pose2map = self.tfBuffer.transform(l_pose_stamped, 'map', timeout=rospy.Duration(1))
                c_pose2map = self.tfBuffer.transform(c_pose_stamped, 'map', timeout=rospy.Duration(1))

            except:
                traceback.print_exc()

#            origin2rx = r_pose_stamped.pose.position.x - c_pose_stamped.pose.position.x
#            origin2ry = r_pose_stamped.pose.position.y - c_pose_stamped.pose.position.y
#            origin2lx = l_pose_stamped.pose.position.x - c_pose_stamped.pose.position.x
#            origin2ly = l_pose_stamped.pose.position.y - c_pose_stamped.pose.position.y
#
#            origin2rxy = origin2rx**2 + origin2ry**2
#            origin2lxy = origin2lx**2 + origin2lx**2
#                     
#            sqrt_r = math.sqrt(origin2rxy)
#            sqrt_l = math.sqrt(origin2lxy)
#
#            print("----------r_hand_distance------------",sqrt_r)
#            print("----------l_hand_distance------------",sqrt_l)
#
#            if sqrt_r > sqrt_l:
#                print("---------Pointing by R hand---------")
#                self.human_coordinate = [r_pose2map.pose.position.x, r_pose2map.pose.position.y, r_pose2map.pose.position.z]
#                print("move to base_footprint coordinate position---->",  self.human_coordinate)
#                
#            elif sqrt_r < sqrt_l:
#                print("---------Pointing by L hand---------")
#                self.human_coordinate = [l_pose2map.pose.position.x, l_pose2map.pose.position.y, l_pose2map.pose.position.z]
#                print("move to base_footprint coordinate position---->", self.human_coordinate)

            ###### vector processing
            r_inner = np.inner(r_elbow_hand,r_reference)
            r_norm = LA.norm(r_elbow_hand) * LA.norm(r_reference)
            r_cos = r_inner / r_norm
            r_deg = np.degrees(r_cos)
            print("r>>",r_deg)
            l_inner = np.inner(l_elbow_hand,l_reference)
            l_norm = LA.norm(l_elbow_hand) * LA.norm(l_reference)
            l_cos = l_inner / l_norm
            l_deg = np.degrees(l_cos)
            print("l>>",l_deg)

            if r_deg > l_deg:
                print("Pointing by R hand")
                self.human_coordinate = [r_pose2map.pose.position.x, r_pose2map.pose.position.y, r_pose2map.pose.position.z]
                return 'success'
            elif r_deg < l_deg:
                print("Pointing by L hand")
                self.human_coordinate = [l_pose2map.pose.position.x, l_pose2map.pose.position.y, l_pose2map.pose.position.z]
                return 'success'

    def execute(self, userdata):
        try:
            start_time = rospy.Time.now()
           
            set_disp = "xhost " + '+'
            start_op = Popen(set_disp, shell=True)
          
            # cmd = "exec " + 'docker-compose -f ~/hsr_ws/src/openpose-ros-docker/pointing-docker-compose.yml up'
            # cmd = "exec " + 'docker-compose -f ~/erasers_ws/src/hsr/openpose-ros-docker/pointing-docker-compose.yml up'
            cmd = "exec " + 'docker compose -f ~/erasers_ws/src/hsr/openpose-ros-docker/pointing-docker-compose.yml up'
            p = Popen(cmd, shell=True)
            print("point_pub is excuted")
            #with TemporarySubscriber('op_result', PoseArray, self.callback):
            sub = rospy.Subscriber('op_result', PoseArray, self.callback)
            while not rospy.is_shutdown():
                if self.human_coordinate:
                    userdata.human_coordinate = self.human_coordinate
                    self.br.sendTransform(tuple(self.human_coordinate), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'pointed_object', "base_footprint")
                    cmd_down = "exec " + \
                            'docker compose -f ~/erasers_ws/src/hsr/openpose-ros-docker/pointing-docker-compose.yml down'
                            # 'docker-compose -f ~/erasers_ws/src/hsr/openpose-ros-docker/pointing-docker-compose.yml down'
                        # 'docker-compose -f ~/hsr_ws/src/openpose-ros-docker/pointing-docker-compose.yml down'
                    p2 = Popen(cmd_down, shell=True)
                    print("------------------- down -------------------")
                    self.human_coordinate = None
                    sub.unregister()
                    return 'success'

                if ((rospy.Time.now() - start_time).to_sec()) > self.timeout:
                    traceback.print_exc()
                    cmd_down = "exec " + \
                        'docker compose -f ~/erasers_ws/src/hsr/openpose-ros-docker/pointing-docker-compose.yml down'
                        # 'docker-compose -f ~/erasers_ws/src/hsr/openpose-ros-docker/pointing-docker-compose.yml down'
                        # 'docker-compose -f ~/hsr_ws/src/openpose-ros-docker/pointing-docker-compose.yml down'
                    p2 = Popen(cmd_down, shell=True)
                    print("------------------- down -------------------")
                    self.human_coordinate = None
                    sub.unregister()
                    return 'timeout'

        except:
            rospy.logerr(traceback.format_exc())
            cmd_down = "exec " + \
                'docker compose -f ~/erasers_ws/src/hsr/openpose-ros-docker/pointing-docker-compose.yml down'
                # 'docker-compose -f ~/erasers_ws/src/hsr/openpose-ros-docker/pointing-docker-compose.yml down'
                # 'docker-compose -f ~/hsr_ws/src/openpose-ros-docker/pointing-docker-compose.yml down'
            p2 = Popen(cmd_down, shell=True)
            print("------------------- down -------------------")
            self.human_coordinate = None
            sub.unregister()
            return 'failure'


if __name__ == "__main__":
    rospy.init_node('smach_example_state_machine')

    sm = smach.StateMachine(outcomes=['success', 'failure'])
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('DEBUG', PointingPerson(),
                               transitions={'success': 'success',
                                            'timeout': 'failure',
                                            'failure': 'failure'})
    sm.execute()
