#!/usr/bin/env python
import roslib; roslib.load_manifest('face_tracking')
import rospy
import sys
import time
import math
from face_tracking.msg import facePosAndSize
#De este solo habra una instancia en qbo_motion_server.py
#Tendra almacenado el estado del robot
#Proporciona las clases para comunicar con ROS

class qbo_face_traking_client():
    def __init__(self):
        #rospy.init_node('qbo_control', anonymous=True)
        self.u=0
        self.v=0
        self.distance=0.0
        self.image_width=0
        self.image_height=0
        self.face_detected=False
        self.type_of_tracking='camshift'

        rospy.Subscriber('/Qbo/FaceImg/PosAndSize', facePosAndSize, self.facePosAndSizeCb, queue_size=1)

    def facePosAndSizeCb(self,msg):
        self.u=msg.u
        self.v=msg.v
        self.distance=msg.distance2head
        self.image_width=msg.image_width
        self.image_height=msg.image_height
        self.face_detected=msg.face_detected
        self.type_of_tracking=msg.type_of_tracking

    def getPosAndSize(self):
        posAndSizeDic={}
        posAndSizeDic['u']=self.u
        posAndSizeDic['v']=self.v
        posAndSizeDic['distance']=self.distance
        posAndSizeDic['image_width']=self.image_width
        posAndSizeDic['image_height']=self.image_height
        posAndSizeDic['face_detected']=self.face_detected
        posAndSizeDic['type_of_tracking']=self.type_of_tracking
        return posAndSizeDic
