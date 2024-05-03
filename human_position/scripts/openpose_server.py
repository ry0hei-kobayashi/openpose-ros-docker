#!/usr/bin/env python3
# coding:UTF-8

import os
import subprocess
import math
import time
import cv2
from cv_bridge import CvBridge
import ros_numpy
import numpy as np
import traceback
from openpose import pyopenpose as op

import rospy
from sensor_msgs.msg import PointCloud2 ,Image
from geometry_msgs.msg import PoseStamped, Pose

import collections
NOSE = 0
NECK = 1
RIGHT_SHOULDER = 2
RIGHT_ELBOW = 3
RIGHT_WRIST = 4
LEFT_SHOULDER = 5
LEFT_ELBOW = 6
LEFT_WRIST = 7
CENTER_HIP = 8
RIGHT_HIP = 9
RIGHT_KNEE = 10
RIGHT_ANKLE = 11
LEFT_HIP = 12
LEFT_KNEE = 13
LEFT_ANKLE = 14
RIGHT_EYE = 15
LEFT_EYE = 16
RIGHT_EAR = 17
LEFT_EAR = 18
PARTS = collections.OrderedDict()
PARTS['NOSE'] = NOSE
PARTS['NECK'] = NECK
PARTS['RIGHT_SHOULDER'] = RIGHT_SHOULDER
PARTS['RIGHT_ELBOW'] = RIGHT_ELBOW
PARTS['RIGHT_WRIST'] = RIGHT_WRIST
PARTS['LEFT_SHOULDER'] = LEFT_SHOULDER
PARTS['LEFT_ELBOW'] = LEFT_ELBOW
PARTS['LEFT_WRIST'] = LEFT_WRIST
PARTS['CENTER_HIP'] = CENTER_HIP
PARTS['RIGHT_HIP'] = RIGHT_HIP
PARTS['RIGHT_KNEE'] = RIGHT_KNEE
PARTS['RIGHT_ANKLE'] = RIGHT_ANKLE
PARTS['LEFT_HIP'] = LEFT_HIP
PARTS['LEFT_KNEE'] = LEFT_KNEE
PARTS['LEFT_ANKLE'] = LEFT_ANKLE
PARTS['RIGHT_EYE'] = RIGHT_EYE
PARTS['LEFT_EYE'] = LEFT_EYE
PARTS['RIGHT_EAR'] = RIGHT_EAR
PARTS['LEFT_EAR'] = LEFT_EAR

class OpenposeServer():
    def __init__(self):

        rospy.init_node('openpose_server', anonymous=True)


        params = dict()
        params["model_folder"] = "/openpose/models/"
        #params["face"] = False
        #params["hand"] = True
        #params["num_gpu"] = 1
        #params["num_gpu_start"] = 1

        #params["net_resolution"] = "-1x192" #nvidia-smi 1746MB
        #params["net_resolution"] = "320x320" #nvidia-smi 2846MB
        #params["net_resolution"] = "640x640" #nvidia-smi 9182MB
        #params["net_resolution"] = "640x640" #nvidia-smi 9182MB
        
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()
        #rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.callback)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.callback)

        self.pub = rospy.Publisher('human_coordinates', PoseStamped, queue_size=10)
        
        self.bridge = CvBridge()
        self.threed_positions = []
        self.point = PointCloud2()


    def callback(self, data):

        self.threed_positions = []

        pcl_data = ros_numpy.numpify(data)
        pcl2img = pcl_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
        bgr2rgb = cv2.cvtColor(pcl2img, cv2.COLOR_BGR2RGB)
        people = self.process_op(bgr2rgb)

        if people is None:
            return

        try:
            for person in people:
                key_dict = collections.OrderedDict()

                for key, key_name in zip(person, PARTS.keys()):

                    pix_x = int(key[0])
                    pix_y = int(key[1])
                    score = key[2]

                    #key_dict[key_name] -> (x, y, z, score)
                    key_dict[key_name] = pcl_data[pix_y][pix_x]
                    #rospy.logwarn(key_dict[key_name])

                    if not math.isnan(pcl_data[pix_y][pix_x][0]) or not math.isnan(pcl_data[pix_y][pix_x][1]) or not math.isnan(pcl_data[pix_y][pix_x][2] ):
                        x = pcl_data[pix_y][pix_x][0]
                        y = pcl_data[pix_y][pix_x][1]
                        z = pcl_data[pix_y][pix_x][2]
                        if x > -4.0 and x < 4.0 and y > -1.3 and z > 0.4 and z < 10:
                            #if person[RIGHT_WRIST][1] == 0:
                            #    person[RIGHT_WRIST][1] = 479

                            #if person[LEFT_WRIST][1] == 0:
                            #    person[LEFT_WRIST][1] = 479

                            rospy.logwarn("%s, %f, %f, %f, %f", key_name, x, y, z, score)


                            #keypoints = {}
                            #for i, key in enumerate(PARTS.keys()):
                            #    keypoints[key] = person[i][0], person[i][1], person[i][2]
                            #        obj = PointStamped()
                            #        obj.header.stamp = rospy.Time.now()
                            #        obj.header.frame_id = data.header.frame_id
                            #        obj.point.x, obj.point.y, obj.point.z = x, y, z                              
                            #        self.pub.publish(obj)
                            #        rospy.logwarn("human_coordinate: %f, %f, %f", obj.point.x ,obj.point.y, obj.point.z)
                            #         print(self.pub.publish(obj))

                    else:
                        continue
                break
        except IndexError:
            pass
        except:
            traceback.print_exc()

    def process_op(self, image):
        datum = op.Datum()
        imageToProcess = image
        datum.cvInputData = imageToProcess
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        try:
            cv2.imshow("WAVING WRIST DETECTOR BY RYOHEISOFT", datum.cvOutputData)
            cv2.waitKey(1)

        except:
            traceback.print_exc()
        return datum.poseKeypoints


if __name__ == '__main__':
    try:
        rospy.logwarn("Start Openpose Server")
        openpose_server = OpenposeServer()
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            #rospy.sleep(1.)
            #rospy.loginfo('Waiting for person founded.')
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

