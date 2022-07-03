#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
#from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int32
from cv_bridge import CvBridge
#import numpy as np 


bridge = CvBridge()

def arucofun(img) :

    key= getattr(aruco , 'DICT_ARUCO_ORIGINAL')
    arucodict=aruco.Dictionary_get(key)
    arucoparam=aruco.DetectorParameters_create()
    bboxs ,ids,rejected =aruco.detectMarkers(img,
                                            arucodict,
                                            parameters=arucoparam)

    aruco.drawDetectedMarkers(img,bboxs)
    if len(bboxs)> 0:
        for bbox,id in zip(bboxs,ids) :
            pub= rospy.Publisher ('id',Int32,queue_size=10)
            tr=(bbox[0][1][0],bbox[0][1][1]) ## top right
            bl=(bbox[0][3][0],bbox[0][3][1]) ## bottom left
            centre=(int((bl[0]+tr[0])/2),int((bl[1]+tr[1])/2))
            pub.publish(id[0])    

            #print(centre) ########### need to be published
            print (type(id[0]))    ########### need to be published 


def callback(img):
    cv_image = bridge.imgmsg_to_cv2(img , "bgr8")
    arucofun(cv_image)
    cv2.waitKey(1)

def listener() :
    rospy.init_node('opencv_image',anonymous=True)   
    image_sub = rospy.Subscriber('/camera_image/image_raw',Image,callback) 
    rospy.spin()
    
        
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
