#!usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from birdeyeview import BEV
#from perspectiveTransform import bev

class Unicon_CV():
    def __init__(self):
        self.camera_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.bridge = CvBridge()
        self.bev = BEV()


    def camera_callback(self, data):
        try:
            # read cameara data, and roi
            cv_image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv_image_raw[0:640][300:480]  

            # bev_tf 
            A = [(251, 13), (400, 13), (580,140), (70,140)]
            bev_image = self.bev.birdeyeview(cv_image,A)

            #preprocessing
            gray = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)  

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
            
            matrix_zeros = np.zeros((10, 32, 2), dtype=int)

            for i in range(5): # line 
                yy = 350 - (i + 1) * 10  # 9간격으로 위로 이동
                for j in range(32):  # 최대로 가로 개수를 늘림
                    #area = img[yy:yy+5, j*20:(j+1)*20]
                    white_pixels = cv2.countNonZero(mask_white[yy:yy+5, j*20:(j+1)*20])
                    yellow_pixels = cv2.countNonZero(mask_yellow[yy:yy+10, j*20:(j+1)*20]) 

                    if white_pixels > 30:  
                        image[yy:yy+4, j*20:(j+1)*20] = [0, 0, 255]  # 
                        matrix_zeros[i][j]=[(j+1)*10,yy+2]
                        
                    elif yellow_pixels > 80: 
                        image[yy:yy+4, j*20:(j+1)*20] = [255, 0, 0]  
                        matrix_zeros[i][j]=[(j+1)*10,yy+2]

            non_zero_values = matrix_zeros[np.all(matrix_zeros != [0, 0], axis=-1)]
            #print(non_zero_values)
            
            #non_zero_list = non_zero_values.tolist()
            #print(non_zero_list)


            cv2.imshow('ROI', cv_image)
            cv2.imshow('BEV', image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User exit with 'q' key")  
                cv2.destroyAllWindows()  
                return

        except CvBridgeError as e:
            print(e)

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node("unicon_cv")
        unicon_cv = Unicon_CV()
        unicon_cv.main()

    except KeyboardInterrupt:
        print("sorry. don't execute")
