#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import sys
import pyrealsense2 as rs
import numpy as np
import cv2
import time

class Detect():
        def __init__(self):
                #################### node initialization & create topic ####################
                rospy.init_node("circle_detector", anonymous=False)
                self.coor_pub = rospy.Publisher("camera/disk_position", Point, queue_size= 30)
                self.img_pub = rospy.Publisher("camera_image/detected", Image, queue_size=5)
                self.flag = Int32()
                self.data = Point()
                self.img = Image()
                self.raw_image = np.asanyarray(0)
                self.color_image = np.asanyarray(0)
                #################### video stream setting ####################
                self.pipeline = rs.pipeline()
                self.config = rs.config()
                self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                # Start streaming
                self.pipeline.start(self.config)
                self.time1 = time.time()
                self.time2 = time.time()
                #################### bounding object configuration ####################
                self.lower_red = np.array([150,100,100])
                self.upper_red = np.array([180,255,255])
                self.contour_area = 0
                self.kernel5 = np.ones((5, 5), np.uint8)
                self.kernel9 = np.ones((9, 9), np.uint8)        
                #################### camera's intrinsic parameter ####################
                self.fx = 893.7527
                self.fy = 895.8022
                self.cx = 648.9854
                self.cy = 374.8394
                self.object_real_width = 0.08
                self.intrinsic_matrix = np.array([[self.fx,       0, self.cx],
                                                [      0, self.fy, self.cy],
                                                                              [      0,       0,       1]])
                try :
                        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)
                except:
                        #sys.exit("intrinsic matrix doesn't have a inverse matrix.")
                        print("intrinsic matrix doesn't have a inverse matrix.")

        def find_contour(self):
                hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
                self.mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
                self.denoise_mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, self.kernel9)
                self.denoise_mask2 = cv2.morphologyEx(self.denoise_mask, cv2.MORPH_CLOSE, self.kernel5, iterations = 3)
                #self.res = cv2.bitwise_and(self.color_image, self.color_image, mask = self.denoise_mask2)
                #self.gray = cv2.cvtColor(self.res, cv2.COLOR_BGR2GRAY)
                #self.blur = cv2.GaussianBlur(self.gray, (5, 5), 0)
                self.binary_img = cv2.Canny(self.denoise_mask2, 20, 160)
                self.contours = cv2.findContours(self.binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]       

        def bound_contour(self):
                if self.contours:
                        for c in self.contours:
                                area = cv2.contourArea(c)
                                if area > self.contour_area:
                                        self.one_contour = c
                                        self.contour_area = area
                        (x, y, self.w, self.h) = cv2.boundingRect(self.one_contour)
                        cv2.rectangle(self.color_image, (x,y), (x+self.w, y+self.h), (0, 255, 0), 2)
                        self.x = x + self.w / 2
                        self.y = y + self.h / 2
                        cv2.circle(self.color_image, (int(self.x), int(self.y)), 8, (1, 277, 254), -1)
                # zero the contour_area
                self.contour_area = 0 

        def check_object(self):
                if self.contours:
                        return True
                else:
                        return False

        def streaming(self):
                frames = self.pipeline.wait_for_frames()
                while frames == 0:
                        frames = self.pipeline.wait_for_frames()

                color_frame = frames.get_color_frame()
                self.color_image = np.asanyarray(color_frame.get_data())
                self.raw_image = self.color_image
        
        def show_result(self):
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow("RealSense", self.color_image)
                #cv2.imshow("mask", self.denoise_mask)
                #cv2.imshow("denoise_mask", self.denoise_mask2)
                #cv2.imshow("canny", self.binary_img)
                #cv2.imshow("blur", self.blur)

        def get_video_size(self):
                #width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                #height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                print("width is %d, height is %d"%(width, height))

        def object_camera_coordinate(self):
                # calculate the camera coordinate by triangle similarity theorem
                camera_coordinate_z = (self.fx / self.w) * self.object_real_width
                camera_coordinate_x = (self.x - self.cx) * camera_coordinate_z / self.fx
                camera_coordinate_y = (self.y - self.cy) * camera_coordinate_z / self.fy
                # record position information
                data = Point()
                data.x = camera_coordinate_x
                data.y = camera_coordinate_y
                data.z = camera_coordinate_z
                print('data: %f  %f  %f', data.x, data.y, data.z)
                return data

        def object_detect(self):
                self.find_contour()
                print('object detect check obj:  %d', d.check_object())
                if self.check_object:
                        self.bound_contour()

        def image_publisher(self):
                gray_img = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2GRAY)
                header = Header(stamp = rospy.Time.now())
                header.frame_id = "object"
                self.img.header = header
                self.img.height = 720
                self.img.width = 1280
                self.img.encoding = "mono8" #"bgr8"#"mono8"
                self.img.step = 320*3
                self.img.data = np.array(gray_img).tobytes()
                self.img_pub.publish(self.img)

        def coordinate_publisher(self):
                print('pub data')
                self.data = self.object_camera_coordinate()
                self.coor_pub.publish(self.data)

        def FPS_estimator(self):
                self.time1 = self.time2
                self.time2 = time.time()
                duration = self.time2 - self.time1
                print("FPS : %d" %(1 / duration))
        def rate(self):
                self.rate.sleep()
                
        def end(self):
                self.pipeline.stop()

if __name__ == "__main__":
        d = Detect()
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
                d.streaming()
                d.object_detect()
                d.image_publisher()
                print('check obj:  %d', d.check_object())
                if d.check_object():
                        d.coordinate_publisher()
                d.FPS_estimator()
                rate.sleep()
                if cv2.waitKey(1) & 0xFF == ord('q'):
                        break   
        d.end()
