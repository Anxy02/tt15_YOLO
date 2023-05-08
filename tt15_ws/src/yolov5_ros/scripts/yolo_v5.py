#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from pathlib import Path
import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np
import pyrealsense2 as rs

from models.experimental import attempt_load
from utils.general import (
    check_img_size, non_max_suppression, scale_coords,
    xyxy2xywh, plot_one_box)
from utils.torch_utils import select_device, time_synchronized
from utils.datasets import letterbox

class Yolo_Dect:
    def __init__(self):
        # ros
        pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')
        self.weights = rospy.get_param('~weight_path', '')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        self.source = '0'
        self.imgsz = 640
        self.conf_thres = rospy.get_param('~conf', '0.6')
        self.iou_thres = rospy.get_param('~iou', '0.3')
        self.classes = [0]
        if (rospy.get_param('/use_cpu', 'false')):
            self.device = 'cpu'
        else:
            self.device = '0'

        self.position_pub = rospy.Publisher(
                pub_topic, BoundingBoxes, queue_size=1)
        self.image_pub = rospy.Publisher(
            '/yolov5/detection_image', Image, queue_size=1)
        
        self.detect()


    def detect(self,save_img=False):
        # Initialize
        device = select_device(self.device)
    
        half = device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        model = attempt_load(self.weights, map_location=device)  # load FP32 model
        self.imgsz = check_img_size(self.imgsz, s=model.stride.max())  # check img_size
        if half:
            model.half()  # to FP16
        # Set Dataloader
        vid_path, vid_writer = None, None
        view_img = True
        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Get names and colors
        names = model.module.names if hasattr(model, 'module') else model.names
        colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

        # Run inference
        t0 = time.time()
        img = torch.zeros((1, 3, self.imgsz, self.imgsz), device=device)  # init img
        _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
        pipeline = rs.pipeline()
        # 创建 config 对象：
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

        # Start streaming
        pipeline.start(config)
        align_to_color = rs.align(rs.stream.color)
        while True:
            self.boundingBoxes = BoundingBoxes()
            start = time.time()
            # Wait for a coherent pair of frames（一对连贯的帧）: depth and color
            frames = pipeline.wait_for_frames()
            frames = align_to_color.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            mask = np.zeros([color_image.shape[0], color_image.shape[1]], dtype=np.uint8)
            mask[0:480, 320:640] = 255

            sources = [self.source]
            imgs = [None]
            path = sources
            imgs[0] = color_image
            im0s = imgs.copy()
            img = [letterbox(x, new_shape=self.imgsz)[0] for x in im0s]
            img = np.stack(img, 0)
            img = img[:, :, :, ::-1].transpose(0, 3, 1, 2)  # BGR to RGB, to 3x416x416, uint8 to float32
            img = np.ascontiguousarray(img, dtype=np.float16 if half else np.float32)
            img /= 255.0  # 0 - 255 to 0.0 - 1.0

            # Get detections
            img = torch.from_numpy(img).to(device)
            if img.ndimension() == 3:
                img = img.unsqueeze(0)
            t1 = time_synchronized()
            pred = model(img, augment='true')[0]

            # Apply NMS
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes , agnostic='true')
            t2 = time_synchronized()

            for i, det in enumerate(pred):  # detections per image
                p, s, im0 = path[i], '%g: ' % i, im0s[i].copy()
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                if det is not None and len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        s += '%g %ss, ' % (n, names[int(c)])  # add to string

                    # Write results
                    for *xyxy, conf, cls in reversed(det):            
                        # ros
                        boundingBox = BoundingBox()
                        boundingBox.probability = np.float64(conf.cpu())
                        boundingBox.xmin = np.float64(xyxy[0].cpu())
                        boundingBox.ymin = np.float64(xyxy[1].cpu())
                        boundingBox.xmax = np.float64(xyxy[2].cpu())
                        boundingBox.ymax = np.float64(xyxy[3].cpu())
                        boundingBox.xmid = np.float64((xyxy[0].cpu() + xyxy[2].cpu()) / 2)
                        boundingBox.ymid = np.float64((xyxy[1].cpu() + xyxy[3].cpu()) / 2)

                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh)  # label format
                        # 计算深度信息
                        distance_list = []
                        mid_pos = [int((int(xyxy[0]) + int(xyxy[2])) / 2), int((int(xyxy[1]) + int(xyxy[3])) / 2)]  # 确定索引深度的中心像素位置左上角和右下角相加在/2
                        min_val = min(abs(int(xyxy[2]) - int(xyxy[0])), abs(int(xyxy[3]) - int(xyxy[1])))  # 确定深度搜索范围
                        randnum = 40
                        for i in range(randnum):
                            bias = random.randint(-min_val // 4, min_val // 4)
                            dist = depth_frame.get_distance(int(mid_pos[0] + bias), int(mid_pos[1] + bias))
                            if dist:
                                distance_list.append(dist)
                        distance_list = np.array(distance_list)
                        distance_list = np.sort(distance_list)[
                                        randnum // 2 - randnum // 4:randnum // 2 + randnum // 4]  # 冒泡排序+中值滤波

                        label = '%s %.2f%s' % (names[int(cls)], np.mean(distance_list), 'm')
                        plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)

                        # ros
                        boundingBox.distance = np.float64(np.mean(distance_list))
                        self.boundingBoxes.bounding_boxes.append(boundingBox)
                        self.position_pub.publish(self.boundingBoxes)
                # 发布ROS图像        
                self.publish_image(im0, 480, 640)
                # cv2.imshow('YOLOv5_show', im0)
                # cv2.waitKey(3)
                    
    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)

def main():
    rospy.init_node('test', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()

if __name__ == '__main__':
    main()
    

    
