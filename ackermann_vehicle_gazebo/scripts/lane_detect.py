#!usr/bin/env python3

import rospy
import numpy as np
import math
#import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from birdeyeview import BEV

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive

from purepursuit import get_delta, get_poly, estimate_line


class Unicon_CV():
    def __init__(self):
        self.camera_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)

        self.bridge = CvBridge()
        self.bev = BEV()

        self.y_repeat = 12
        self.right_x = 0
        self.left_x = 0
        self.point_scale = 10

        self.left_lane_pred = np.zeros((self.y_repeat,2))
        self.right_lane_pred = np.zeros((self.y_repeat,2))

    def camera_setting(self,data):
        # read cameara data, and ROI
        cv_image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv_image_raw[0:640][300:480]  

        # bev_tf  
        A = [(251, 13), (400, 13), (580,140), (70,140)] # two line
        bev_image = self.bev.birdeyeview(cv_image, A)


        hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)

        # white mask hsv lange 
        lower_white = np.array([0, 0, 85])
        upper_white = np.array([179, 25, 255])

        # yello mask hsv lange
        lower_yellow = np.array([20, 100, 70])
        upper_yellow = np.array([30, 255, 255])

        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        image = bev_image.copy()

        return mask_white, mask_yellow, image
    
    def find_certain_color(self,mask_certain,threshold,j):
        certain_pixels = cv2.countNonZero(mask_certain[self.yy:self.yy+5, j*10:(j+1)*10])

        if certain_pixels > threshold:
            return [(j+1)*10,self.yy+2]
        else:
            return []

    def camera_callback(self, data):
        try:
            mask_white, mask_yellow, image = self.camera_setting(data)
            
            for i in range(self.y_repeat):
                self.yy = 430 - (i + 1) * self.point_scale # point_scale 간격으로 위로 이동
                self.left_x = 0
                self.right_x = 0

                for j in range(64):     
                    self.left_lane_points = self.find_certain_color(mask_white,15,j)      # x for white line
                    self.right_lane_points = self.find_certain_color(mask_yellow,35,j)    # x for yellow line

                    if len(self.left_lane_points) > 0:
                        self.left_x = self.left_lane_points[0]
                    if len(self.right_lane_points) > 0:
                        self.right_x = self.right_lane_points[0]

                self.left_lane_pred[i][0] = self.left_x
                self.left_lane_pred[i][1] = self.yy+2
                image[self.yy:self.yy+5, self.left_x-20:self.left_x] = [255, 0, 0]

                self.right_lane_pred[i][0] = self.right_x
                self.right_lane_pred[i][1] = self.yy+2
                image[self.yy:self.yy+5, self.right_x-20:self.right_x] = [0, 0, 255]

            steering_angle = estimate_line(self.y_repeat,self.left_lane_pred,self.right_lane_pred,image)
                
            cv2.imshow('BEV', image)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User exit with 'q' key")  
                cv2.destroyAllWindows()  
                return

            move_cmd.linear.x = 0.7
            move_cmd.angular.z = steering_angle

            cmd_vel_pub.publish(move_cmd)
            
        except CvBridgeError as e:
            print(e)

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        move_cmd = Twist()
        
        rospy.init_node("lane_detect")
        unicon_cv = Unicon_CV()
        unicon_cv.main()

    except KeyboardInterrupt:
        print("sorry. don't execute")
        
