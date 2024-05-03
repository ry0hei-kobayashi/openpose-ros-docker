#!/usr/bin/env python
# coding:UTF-8

import os
import subprocess
import math
import time
import cv2
from cv_bridge import CvBridge
import ros_numpy
import numpy as np
import collections
import traceback
from openpose import pyopenpose as op
#from common import speech

import rospy
from sensor_msgs.msg import PointCloud2
from human_position.msg import Object
from geometry_msgs.msg import PointStamped

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
PARTS = collections.OrderedDict()
PARTS['NOSE'] = NOSE
PARTS['NECK'] = NECK
PARTS['R_SHOULD'] = R_SHOULD
PARTS['R_ELBOW'] = R_ELBOW
PARTS['R_HAND'] = R_HAND
PARTS['L_SHOULD'] = L_SHOULD
PARTS['L_ELBOW'] = L_ELBOW,
PARTS['L_HAND'] = L_HAND
PARTS['C_WAIST'] = C_WAIST
PARTS['R_WAIST'] = R_WAIST
PARTS['R_NEE'] = R_NEE
PARTS['R_FOOT'] = R_FOOT
PARTS['L_WAIST'] = L_WAIST,
PARTS['L_NEE'] = L_NEE
PARTS['L_FOOT'] = L_FOOT,
PARTS['R_EYE'] = R_EYE
PARTS['L_EYE'] = L_EYE
PARTS['R_EAR'] = R_EAR
PARTS['L_EAR'] = L_EAR


class Openpose:
    def __init__(self):
        rospy.init_node('op_publisher', anonymous=True)

        #process = subprocess.Popen(['ps', '-a'], stdout=subprocess.PIPE)
        #output, error = process.communicate()
        #for line in output.splitlines():
        #    if "python3" in str(line):
        #        pid = int(line.split(None, 1)[0])
        #        os.kill(pid, 9)

        params = dict()
        params["model_folder"] = "/openpose/models/"
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()
        rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.callback)
        self.pub = rospy.Publisher('op_result', Object, queue_size=10)
        self.threed_positions = []
        self.point = PointCloud2()
        self.f_count = 0
        self.rec_human = 0
        self.rec_count = 0
        self.rec_stamp = 0

    def callback(self, data):
        # print("cb")
        self.threed_positions = []
        now_time = rospy.Time.now()
        if (now_time - data.header.stamp).to_sec() > 1.5:  # 誤差1.5s以上なら飛ばす
            print(now_time - data.header.stamp).to_sec()
            print("time is far from now")
            return

        point_data = ros_numpy.numpify(data)
        # print(point_data.shape)
        image_data = point_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
        image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
        people = self.process_op(image_data)

        if people is None:
            return

        try:
            for person in people:
                threed_position = collections.OrderedDict()
                if self.rec_count >= 2:  # 前回見つけた人を2フレーム連続で見つけられなかったらリセットする
                    self.rec_human = 0
                    self.rec_count = 0
                    self.f_count = 0
                    self.rec_stamp = 0

                if self.f_count > 0:
                    # 前回見つけた手を挙げている人のみを見る #40ピクセルより小さいものは検出しない
                    if self.rec_human != 0 and self.rec_human-person[NECK][0] < 40 and self.rec_human-person[NECK][0] > -40:
                        pass
                    else:
                        self.rec_count += 1
                        continue

                row = int(person[C_WAIST][0])
                col = int(person[C_WAIST][1])

                if not math.isnan(point_data[col][row][0]) or not math.isnan(point_data[col][row][1]) or not math.isnan(point_data[col][row][2]):
                    x = point_data[col][row][0]
                    y = point_data[col][row][1]
                    z = point_data[col][row][2]
                    if x > -1.4 and x < 1.4 and y > -1.3 and z > 0.4 and z < 2.5:
                        if person[R_HAND][1] == 0:
                            person[R_HAND][1] = 479

                        if person[L_HAND][1] == 0:
                            person[L_HAND][1] = 479

                        if (int(person[NECK][1]) < int(person[C_WAIST][1])) and person[C_WAIST][1]-person[NECK][1] > 20:
                            self.f_count += 1
                            self.rec_human = person[NECK][0]
                            if self.rec_stamp == 0:
                                self.rec_stamp = data.header.stamp

                            print("count:{:>3} ,time:{:>2}".format(self.f_count, abs(
                                (data.header.stamp-self.rec_stamp).to_sec())))

                            if self.f_count >= 2 and (abs((data.header.stamp-self.rec_stamp).to_sec()) >= 0.5):
                                self.f_count = 0
                                self.rec_stamp = 0
                                push_PARTS = {}
                                for i, key in enumerate(PARTS.keys()):
                                    push_PARTS[key] = person[i][0], person[i][1], person[i][2]

                                rospy.loginfo(
                                    'I found a person who raised hand!')

                                obj = Object()
                                obj.center.header.stamp = rospy.Time.now()
                                obj.center.header.frame_id = data.header.frame_id
                                obj.center.point.x, obj.center.point.y, obj.center.point.z = x, y, z
                                self.pub.publish(obj)
                                #self.pub.publish(obj.center.point.x, obj.center.point.y, obj.center.point.z)
                                print(obj.center.point.x,
                                      obj.center.point.y, obj.center.point.z)
                                # print(self.pub.publish(obj))

                        else:
                            self.f_count = 0
                            self.rec_stamp = 0
                            break
                else:
                    continue
                break
        except IndexError:
            pass

    def process_op(self, image):
        # print("process")
        datum = op.Datum()
        imageToProcess = image
        datum.cvInputData = imageToProcess
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        try:
            cv2.imshow("OpenPose_pict", datum.cvOutputData)
            cv2.waitKey(1)

        except:
            pass
        return datum.poseKeypoints


if __name__ == '__main__':
    try:
        print("Start Openpose.")
        openpose = Openpose()
        rate = rospy.Rate(1000)

        while not rospy.is_shutdown():
            rospy.sleep(1.)
            rospy.loginfo('Waiting for person founded.')
    except rospy.ROSInterruptException:
        pass
