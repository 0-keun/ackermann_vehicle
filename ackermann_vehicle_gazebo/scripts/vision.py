#!usr/bin/env python3

import rospy
import numpy as np
#import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from birdeyeview import BEV

class Unicon_CV():
    def __init__(self):
        self.camera_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.bridge = CvBridge()
        self.bev = BEV()
        self.left_lane_points = []
        self.right_lane_points = []
        self.avg_x = []
        self.prev_yy_check = None

    def camera_callback(self, data):
        try:
            # read cameara data, and roi
            cv_image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv_image_raw[0:640][300:480]  

            # bev_tf 
            A = [(251, 13), (400, 13), (580,140), (70,140)]
            bev_image = self.bev.birdeyeview(cv_image, A)

            #annotation beacuse don't use
            #gray = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
            #blur = cv2.GaussianBlur(gray, (5, 5), 0)  

            hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)

            # white mask hsv lange 
            lower_white = np.array([0, 0, 180])
            upper_white = np.array([255, 30, 255])

            # yello mask hsv lange
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])

            mask_white = cv2.inRange(hsv, lower_white, upper_white)
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

            image = bev_image.copy()
            
            #matrix_zeros = np.zeros((10, 32, 2), dtype=int)
            point_scale = 35
            for i in range(10): # line each 
                yy = 430 - (i + 1) * point_scale # point_scale 간격으로 위로 이동
                
                for j in range(32):  # 최대로 가로 개수를 늘림
                    #area = img[yy:yy+5, j*20:(j+1)*20]
                    white_pixels = cv2.countNonZero(mask_white[yy:yy+5, j*20:(j+1)*20]) # y , x 
                    yellow_pixels = cv2.countNonZero(mask_yellow[yy:yy+5, j*20:(j+1)*20])  # y , x

                    if white_pixels > 30:  
                        image[yy:yy+5, j*20:(j+1)*20] = [0, 0, 255]
                        #cv2.circle(image, ((j+1)*32, yy+2), 5, (0, 0, 255), -1)
                        #matrix_zeros[i][j]=[(j+1)*20,yy+2]
                        self.left_lane_points.append([(j+1)*20,yy+2])
                        if len(self.left_lane_points) > 5:
                            del self.left_lane_points[0]

                    if yellow_pixels > 80: 
                        image[yy:yy+5, j*20:(j+1)*20] = [255, 0, 0]  
                        #cv2.circle(image, ((j+1)*32, yy+2), 5, (255, 0, 0), -1)
                        #matrix_zeros[i][j]=[(j+1)*20,yy+2]
                        self.right_lane_points.append([(j+1)*20,yy+2])
                        if len(self.right_lane_points) > 5:
                            del self.right_lane_points[0]

                    if len(self.left_lane_points) > 0 and len(self.right_lane_points) > 0:  #left, right detect check
                        
                        if len(self.avg_x) > 4: # i want five points
                            del self.avg_x[0]

                        left_x = self.left_lane_points[0][0] 
                        right_x = self.right_lane_points[0][0]

                        avg_x_val = int((left_x + right_x) / 2)
                        self.avg_x.append(avg_x_val)

                        if self.prev_yy_check != yy:
                            print(f"yy : {yy}")
                            cv2.circle(image, (self.avg_x[0], yy), 5, (0, 255, 0), -1) # radius 5
                            self.prev_yy_check = yy

                            x_data = np.array(self.avg_x) # center_x coordinate
                            y_data = np.array([yy] * len(self.avg_x)) # center_y coordinate 

                            order = 4 # 4-order equation
                            coefficients = np.polyfit(x_data, y_data, order) # parameter [c3, c2, c1, c0] sequence print
                            print(f"Co : {coefficients}")
         
            print(f"left : {self.left_lane_points}")
            print(f"right : {self.right_lane_points}")
            print(f"center : {self.avg_x}")
            cv2.imshow('ROI', cv_image)
            cv2.imshow('BEV', image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User exit with 'q' key")  
                cv2.destroyAllWindows()  
                return

        except CvBridgeError as e:
            print(e)


    #def draw_lane(self):
        # Use matplotlib 
        # draw_lane
         
    def main(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        rospy.init_node("unicon_cv")
        unicon_cv = Unicon_CV()
        unicon_cv.main()

    except KeyboardInterrupt:
        print("sorry. don't execute")