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

from purepursuit import get_theta, get_coef_oneside, get_poly


class Unicon_CV():
    def __init__(self):
        self.camera_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.bridge = CvBridge()
        self.bev = BEV()
        self.y_repeat = 12
        self.right_x = 0
        self.left_x = 0
        self.left_lane_pred = np.zeros((self.y_repeat,2))
        self.right_lane_pred = np.zeros((self.y_repeat,2))
        self.avg_pred = np.zeros((self.y_repeat,2))
        self.prev_yy_check = None
        self.center_y = []

    def camera_callback(self, data):
        try:
            # read cameara data, and roi
            cv_image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv_image_raw[0:640][300:480]  

            # bev_tf  
            A = [(251, 13), (400, 13), (580,140), (70,140)] # two line
            #A = [(120, 31), (510, 31), (620,69), (18,69)] # one line
            bev_image = self.bev.birdeyeview(cv_image, A)


            hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)

            # white mask hsv lange 
            lower_white = np.array([0, 0, 180])
            upper_white = np.array([179, 25, 255])

            # yello mask hsv lange
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])

            mask_white = cv2.inRange(hsv, lower_white, upper_white)
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

            image = bev_image.copy()
            
            point_scale = 10
            for i in range(self.y_repeat): # line each 
                yy = 430 - (i + 1) * point_scale # point_scale 간격으로 위로 이동
                self.left_x = 0
                self.right_x = 0
                for j in range(64):  # 최대로 가로 개수를 늘림
                    self.left_lane_points = []
                    self.right_lane_points = []
                    
                    white_pixels = cv2.countNonZero(mask_white[yy:yy+5, j*10:(j+1)*10]) # y , x 
                    yellow_pixels = cv2.countNonZero(mask_yellow[yy:yy+5, j*10:(j+1)*10])  # y , x

                    if white_pixels > 15:  
                        # image[yy:yy+5, j*20:(j+1)*20] = [0, 0, 255]
                        self.right_lane_points = [(j+1)*10,yy+2]

                    if yellow_pixels > 35: #yello extraction 
                        # image[yy:yy+5, j*20:(j+1)*20] = [255, 0, 0]  
                        self.left_lane_points = [(j+1)*10,yy+2]

                        # if self.prev_yy_check != yy:
                        #     #print(f"yy : {yy}")
                        #     cv2.circle(image, (self.avg_x[0], yy), 5, (0, 255, 0), -1) # radius 5
                        #     self.prev_yy_check = yy

                    if len(self.left_lane_points) > 0:
                        self.left_x = self.left_lane_points[0]

                    if len(self.right_lane_points) > 0:
                        self.right_x = self.right_lane_points[0]

                self.left_lane_pred[i][0] = self.left_x
                self.left_lane_pred[i][1] = yy+2
                image[yy:yy+5, self.left_x-20:self.left_x] = [255, 0, 0]

                self.right_lane_pred[i][0] = self.right_x
                self.right_lane_pred[i][1] = yy+2
                image[yy:yy+5, self.right_x-20:self.right_x] = [0, 0, 255]

            right_count = np.count_nonzero(self.right_lane_pred) - self.y_repeat
            left_count = np.count_nonzero(self.left_lane_pred) - self.y_repeat

            xy_l = np.zeros((2,left_count))
            xy_r = np.zeros((2,right_count))

            if right_count > 2:
                r_coef = get_coef_oneside(self.y_repeat,self.right_lane_pred,xy_r)
            if left_count > 2:
                l_coef = get_coef_oneside(self.y_repeat,self.left_lane_pred,xy_l)

            if right_count > 2 and left_count > 2:
                coef = [0,0]
                coef[0] = (l_coef[0] + r_coef[0])/2
                coef[1] = (l_coef[1] + r_coef[1])/2
                for i in range(self.y_repeat): 
                    yy = 430 - (i + 1) * point_scale
                    poly = get_poly(coef,yy)
                    tem = int(poly)
                    image[yy:yy+5, tem-10:tem] = [0, 255, 0]

            elif right_count > 2 and left_count < 3:
                coef = [0,0]
                coef[0] = r_coef[0]
                coef[1] = -(442*r_coef[0]) + 225
                for i in range(self.y_repeat):
                    yy = 430 - (i + 1) * point_scale
                    poly = get_poly(coef,yy)
                    tem = int(poly)
                    image[yy:yy+5, tem-10:tem] = [0, 255, 0]

            elif left_count > 2 and right_count < 3:
                coef = [0,0]
                coef[0] = l_coef[0]
                coef[1] = -(442*l_coef[0]) + 225
                for i in range(self.y_repeat):
                    yy = 430 - (i + 1) * point_scale
                    poly = get_poly(coef,yy)
                    tem = int(poly)
                    image[yy:yy+5, tem-10:tem] = [0, 255, 0]

            else:
                coef = [0,0]
                coef[0] = 0
                coef[1] = 225



            cv2.imshow('ROI', cv_image)
            cv2.imshow('BEV', image)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User exit with 'q' key")  
                cv2.destroyAllWindows()  
                return

            steering_angle = get_theta(coef)
        
            if steering_angle > 0.13:
                move_cmd.linear.x = 0.5
                move_cmd.angular.z = min(steering_angle,0.8)

            elif steering_angle < -0.13: 
                move_cmd.linear.x = 0.5
                move_cmd.angular.z = max(steering_angle,-0.8)

            else:
                move_cmd.linear.x = 0.5
                move_cmd.angular.z = 0

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
 