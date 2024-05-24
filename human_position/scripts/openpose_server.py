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
        
        ####################
        # アクション認識をデフォルトで起動しないように
        self.is_waving_hand_recogniion = True
        # self.is_waving_hand_recogniion = rospy.get_param("~is_wavinghand_recognition", False)
        
        # フレーム分msgを保存しておく必要がある
        self.frames_human_coordinates_array = []
        self.required_frames = 3  # 蓄積するフレーム数
        ####################


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
                            hc.waving_score = 0.0

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
                

            # rospy.loginfo(hca)
            self.pub.publish(hca)
            
            #########################
            '''
            Action Recognition
            - 最低でも3フレーム分は回したいので msg の更新が必要不可欠
            - 1フレーム分のwaving_th(手首の変化量)をclient側に送ってもいいが, それだと手フリ認識サーバとは言えないね
                - 手フリ認識(フレーム分回してwaving_thを超えた回数で手振りとみなす)はclient側で書く必要がある
                - だとしたらサーバ側にwaving_thを持ってくる意味あんまない
                
            -> self.frames_human_coordinates_array = [] でフレーム分msgを保存すれば使えるかも
            '''
            #########################
            if self.is_waving_hand_recogniion:
                # for i in range(self.flame_num):
                    # hcaのmsgはフレームごとに更新されるのか？
                    #self.run_waving_hand_recognition(self, hca) # waving_th をreturn
                # self.run_waving_hand_recognition(self, hca) # waving_th をreturn
                
                self.frames_human_coordinates_array.append(hca)
                if len(self.frames_human_coordinates_array) >= self.required_frames:
                    self.waving_th = self.run_waving_hand_recognition(self.frames_human_coordinates_array)
                    self.frames_human_coordinates_array = []
                    
                hc.waving_score = self.waving_th
                
            rospy.loginfo(hc)         
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
    
    def calculate_distance(self, keypoint_1: Point, keypoint_2: Point):
        x = keypoint_1.x - keypoint_2.x
        y = keypoint_1.y - keypoint_2.y
        z = keypoint_1.z - keypoint_2.z
        distance = math.sqrt(x**2 + y**2 + z**2)
        return distance
    
    def run_waving_hand_recognition(self, frames):
        if self.is_waving_hand_recogniion is False:
            return
        
        # 手振りのしきい値のリストを作成
        waving_th_list = []
        
        # 手振りのしきい値を設定
        self.sholder_th = 0.2  # キーポイントの信頼値に関するしきい値（お客さんを探している最中に変化する）
        self.shoulder_distance_th = 0.2
        self.wrist_th = 0.3
        
        # お客さんの番号
        self.customer_number = 0
        # ラベル付きのお客さんの座標記録dict
        self.coordinate_labels = {}
        self.saved_keypoints_labels = {}
        self.saved_keypoints = {}
        
        # 蓄積された全てのフレームの人間の座標データを処理
        for frame in frames:
            rospy.logdebug("start waving hand recognition")
            for detect_person in frame.human_coordinates_array:
                try:
                    keypoints = {kpt.name:kpt for kpt in detect_person.keypoints if kpt.name in ["LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_WRIST", "RIGHT_WRIST"]}
                    # rospy.logwarn(keypoints)

                    right_shoulder = keypoints["RIGHT_SHOULDER"]
                    left_shoulder = keypoints["LEFT_SHOULDER"]

                    right_wrist = keypoints["RIGHT_WRIST"]
                    left_wrist = keypoints["LEFT_WRIST"]

                    shoulder_distance = self.calculate_distance(left_shoulder.point, right_shoulder.point)
                    
                    ### TODO: 信頼値が低い結果(scoreが低い，両肩が離れすぎている)ははじく
                    if (left_shoulder.score < self.sholder_th + 0.1) or (right_shoulder.score < self.sholder_th + 0.1) or (shoulder_distance > 1.0):
                        rospy.logdebug("shoulder____skip")
                        continue
                    # 手首の信頼度が低いものもはじく
                    if (left_wrist.score < self.wrist_th + 0.1) and (right_wrist.score < self.wrist_th + 0.1):
                        rospy.logdebug("wrist____skip")
                        continue
                    
                    # 手を挙げている人を見つけるとLookON
                    if left_shoulder.point.y > left_wrist.point.y or right_shoulder.point.y > right_wrist.point.y:
                        
                        # 信頼値の高い結果が得られた場合に，その人の位置を計算する
                        self.center_point = Point(
                            (left_shoulder.point.x + right_shoulder.point.x) / 2.0,
                            (left_shoulder.point.y + right_shoulder.point.y) / 2.0,
                            (left_shoulder.point.z + right_shoulder.point.z) / 2.0
                        )
                    
                    # １ループ目
                    if not self.coordinate_labels:
                        saved_keypoints = {
                            "left_wrist": left_wrist,
                            "right_wrist": right_wrist
                        }
                        # 座標に番号を付ける
                        #######################################################################
                        self.coordinate_labels[self.customer_number] = self.center_point
                        self.saved_keypoints_labels[self.customer_number] = saved_keypoints
                        self.customer_number += 1
                        continue
                    # ２ループ目以降
                    else:
                        for key, value in self.coordinate_labels.items():
                            # もし取得した center_pointのx座標が既に coordinate_labelsのx, z座標の±0.5m内に存在する場合、同じ人が挙手しているとみなす
                            if value.x - 0.5 < self.center_point.x < value.x + 0.5:
                                if value.z - 0.5 < self.center_point.z < value.z + 0.5:
                                    ### TODO: 信頼値が低い結果(scoreが低い，両肩が離れすぎている)ははじく
                                    # if (left_shoulder.score < self.sholder_th) or (right_shoulder.score < self.sholder_th) or (shoulder_distance > 1.0):
                                    #     rospy.logdebug("shoulder____skip")
                                    #     continue
                                    # # 手首の信頼度が低いものもはじく
                                    # if (left_wrist.score < self.wrist_th) and (right_wrist.score < self.wrist_th):
                                    #     rospy.logdebug("wrist____skip")
                                    #     continue
                                    
                                    # recoginize to waving hand
                                    saved_keypoints = self.saved_keypoints_labels[key]
                                    # rospy.logdebug(saved_keypoints["left_wrist"].point.x - left_wrist.point.x)
                                    rospy.logdebug(saved_keypoints["right_wrist"].point.x - right_wrist.point.x)
                                    print("肩の距離")
                                    print(shoulder_distance)
                                    print("閾値：")
                                    print(abs(saved_keypoints["right_wrist"].point.x - right_wrist.point.x) / (shoulder_distance))
                                    
                                    # 左右の手振りの信頼度
                                    self.left_waving_th = abs(saved_keypoints["left_wrist"].point.x - left_wrist.point.x) / (shoulder_distance) 
                                    self.right_waving_hand = abs(saved_keypoints["right_wrist"].point.x - right_wrist.point.x) / (shoulder_distance)
                                    # 左右で高い方を手振りのしきい値に設定する
                                    self.waving_th = max(self.left_waving_th, self.right_waving_hand)
                                    
                                    # waving_thをリストに追加
                                    waving_th_list.append(self.waving_th)
                except KeyError:
                    pass
                except:
                    traceback.print_exc()
        # waving_thの平均値を計算して返す
        if waving_th_list:
            average_waving_th = sum(waving_th_list) / len(waving_th_list)
        else:
            average_waving_th = 0.0

        return average_waving_th   
                                
if __name__ == '__main__':
    try:
        rospy.logwarn("Start Openpose Server")
        openpose_server = OpenposeServer()
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

