# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 15:17:33 2020

@author: oscar
"""

#!/usr/bin/env python3
    
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from std_msgs.msg import String


import os
import sys
print (sys.path  )
######cv2 conflicts with ros, so remove from path after importing what is needed
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

print (sys.path)
import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
kill_node=False
cap_cnt=0

def callback(img_msg):
    global kill_node , cap_cnt
    

    cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    

    cv2.namedWindow("hand_cam")

    cv2.imshow("hand_cam", cv2_img)    
    
    k = cv2.waitKey(122)
    if k & 0xFF == ord('q'):
        # q key pressed so quit
        print("Quitting...")
        kill_node=True
        exit()
        

    elif k & 0xFF == ord('c'):
        # c key pressed so capture frame to image file
                
        cap_cnt=cap_cnt+1
        cap_name = "imagesfromlistener/image_from_listener_{}.png".format(cap_cnt)
        cv2.imwrite(cap_name, cv2_img)
        print("Saving {}!".format(cap_name))
        

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global kill_node
    print(kill_node)
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, callback)
    
    #rospy.Subscriber("/hsrb/hand_camera/image_raw", Image, callback)
    #rospy.Subscriber("/hsrb/base_scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    if not kill_node:
        rospy.spin()
    else:
        cv2.destroyAllWindows()
        exit()

if __name__ == '__main__':
    listener()
