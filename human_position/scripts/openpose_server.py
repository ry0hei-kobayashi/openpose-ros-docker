#!/usr/bin/env python3
# coding:UTF-8

import math
import numpy as np
import traceback

import collections
from openpose import pyopenpose as op

import cv2
from cv_bridge import CvBridge

import rospy
import ros_numpy

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from human_position.msg import HumanCoordinatesArray, HumanCoordinates, Keypoint


### ############## ###
### keypoints dict ###
### ############## ###
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

        #########
        #if you want to use some 'params' for openpose, pls add here
        params = dict()
        params["model_folder"] = "/openpose/models/"
        #params["face"] = False
        #params["hand"] = True
        #params["num_gpu"] = 1
        #params["num_gpu_start"] = 1

        #params["net_resolution"] = "-1x192" #nvidia-smi 1746MB
        #params["net_resolution"] = "320x320" #nvidia-smi 2846MB
        #params["net_resolution"] = "640x640" #nvidia-smi 9182MB
        #########

        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()

        #debugging for hsrb
        rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.callback)

        #debugging for xtion
        #rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.callback)
        self.pub = rospy.Publisher('human_coordinates', HumanCoordinatesArray, queue_size=10)
        self.bridge = CvBridge()


    def callback(self, data):

        pcl_data = ros_numpy.numpify(data)
        pcl2img = pcl_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
        bgr2rgb = cv2.cvtColor(pcl2img, cv2.COLOR_BGR2RGB)
        people = self.proc_openpose2img(bgr2rgb)

        if people is None:
            return

        try:

            hca = HumanCoordinatesArray()
            hca.header.stamp = rospy.Time.now()
            hca.header.frame_id = data.header.frame_id
            hca.number_of_people = len(people)

            for person in people:
                hc = HumanCoordinates()

                for key, key_name in zip(person, PARTS.keys()):

                    pix_x = int(key[0])
                    pix_y = int(key[1])
                    score = key[2]

                    #PARTS[key_name]: pix_x, pix_y -> x, y, z, score
                    PARTS[key_name] = pcl_data[pix_y][pix_x]

                    if not math.isnan(pcl_data[pix_y][pix_x][0]) or not math.isnan(pcl_data[pix_y][pix_x][1]) or not math.isnan(pcl_data[pix_y][pix_x][2] ):
                        x = pcl_data[pix_y][pix_x][0]
                        y = pcl_data[pix_y][pix_x][1]
                        z = pcl_data[pix_y][pix_x][2]
                        if x > -4.0 and x < 4.0 and y > -1.3 and z > 0.4 and z < 10:
                            kp = Keypoint()
                            kp.name = key_name
                            p = Point()
                            p.x, p.y, p.z = x, y, z
                            # kp.coordinate = p
                            kp.point = p
                            kp.score = score

                            #hc is a whole body keypoints.
                            hc.keypoints.append(kp)


                            ###for bbox
                            KEYPOINTS = {}
                            for i, key in enumerate(PARTS.keys()):
                                KEYPOINTS[key] = person[i][0], person[i][1], person[i][2]

                            ###bbox###
                            col_array = []
                            row_array = []
                            for i in range(len(person)):
                               col_array.append(person[i][0]) 
                               row_array.append(person[i][1]) 
                            col_array = [i for i in col_array if i != 0]
                            row_array = [i for i in row_array if i != 0]

                            #hc is a person bbox.
                            hc.x = int(max(col_array))
                            hc.w = int(min(col_array))
                            hc.y = int(max(row_array))
                            hc.h = int(min(row_array))

                            #print(col_max,col_min,row_max,row_min)
                            #cv2.rectangle(bgr2rgb,(col_max,row_max),(col_min , row_min), (0,255,0),2)
                            #img = self.bridge.cv2_to_imgmsg(bgr2rgb, encoding="bgr8")
                            #self.detected_human_pub.publish(img)                         
                            #cv2.imshow("WAVINGHAND DETECTED",bgr2rgb)
                            #cv2.waitKey(50)

                    else:
                        continue

                #hca has multiple person's keypoints
                hca.human_coordinates_array.append(hc)

            rospy.loginfo(hca)
            self.pub.publish(hca)
            
        except IndexError:
            pass
        except:
            traceback.print_exc()


    def proc_openpose2img(self, image):
        datum = op.Datum()
        imageToProcess = image
        datum.cvInputData = imageToProcess
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        try:
            cv2.imshow("HUMAN POSE DETECTOR BY RYOHEISOFT", datum.cvOutputData)
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
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

