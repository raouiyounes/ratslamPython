#! /usr/bin/env python

import roslib
#roslib.manifest('package')
import sys
import numpy
import numpy as np
import rospy
import cv2

import ros_numpy
import ratslam.local_view_match
from std_msgs.msg import String
from node_example.msg import ViewTemplate   
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import  Image
from sensor_msgs.msg import PointCloud2 
from cmath import sqrt
lv=ratslam.local_view_match.LocalViewMatch()
topic_root='irat_red'


def image_callback(image,pub_vt):
    

    imdatatest=np.zeros(image.width*image.height*3)
    imdata=ros_numpy.numpify(image)
    k=0
    
    for i in range(image.height):
        for j in range(image.width):
            imdatatest[k]=imdata[i][j][0]
            imdatatest[k+1]=imdata[i][j][1]
            imdatatest[k+2]=imdata[i][j][2]
            k+=3
            #k+=1
    
    lv.on_image(imdatatest,False,image.width,image.height)
    vt_output=ViewTemplate()
    vt_output.header.stamp=rospy.Time.now()
    vt_output.header.seq+=1
    vt_output.current_id=lv.get_current_vt()
    vt_output.relative_rad=lv.get_relative_rad()
    pub_vt.publish(vt_output)
def listener():
    
    #pub_vt.publish(vt_output)
    topic_root=""
    lv=ratslam.local_view_match
    
    pub_vt=rospy.Publisher("/irat_red/LocalView/Template",ViewTemplate,queue_size=0)
    sub=rospy.Subscriber("/irat_red/camera/image",Image,image_callback,pub_vt)
    #sub=rospy.Subscriber('/irat_red/camera/image',Image,image_callback,pub_vt)
  
    rospy.spin()
if __name__=='__main__':
    rospy.init_node('RatSLAMLocalViewCells',anonymous=True)
    listener()
    
    
    
    
    
    