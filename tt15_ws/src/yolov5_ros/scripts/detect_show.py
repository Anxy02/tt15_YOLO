#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes

class Show:
    def __init__(self):
        self.box_sub = rospy.Subscriber(
            "/yolov5/BoundingBoxes", BoundingBoxes, self.box_callback)#订阅识别类别话题
        self.color_sub = rospy.Subscriber('/yolov5/detection_image', Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)
        
    def image_callback(self,image):
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        cv2.imshow('YOLOv5_show', self.color_image)
        cv2.waitKey(3)

    def box_callback(self,msg):
        count=0            
        for i in msg.bounding_boxes:
            count+=1

        for tmp_box in msg.bounding_boxes:
            Xmin=tmp_box.xmin
            Xmax=tmp_box.xmax
            Ymin=tmp_box.ymin
            Ymax=tmp_box.ymax
            # label=tmp_box.Class
            probability=tmp_box.probability
            distance=tmp_box.distance
            print("Xmin:",Xmin,"Xmax:",Xmax,"Ymin:",Ymin,"Ymax:",Ymax,"distance:",distance)



def main():
    rospy.init_node('show')
    people_show = Show()
    rospy.spin()
if __name__ == '__main__':
    main()

# Header header
# Header image_header