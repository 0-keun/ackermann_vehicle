import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from birdeyeview import birdeyeview

class Unicon_CV():
    def __init__(self):
        self.camera_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.bridge = CvBridge()

    def camera_callback(self, data):
        try:
            cv_image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv_image_raw[0:640][240:480] 

            A = [(398, 354), (581, 345), (909, 463), (94, 505)] ## 알맞게 조정할 것
            birdeyevie_ = birdeyeview(cv_image,A)

            gray = cv2.cvtColor(birdeyevie_, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)  

            hsv = cv2.cvtColor(cv_image_raw, cv2.COLOR_BGR2HSV)

            # 흰색에 대한 HSV 범위
            lower_white = np.array([0, 0, 180])
            upper_white = np.array([255, 30, 255])

            # 노란색에 대한 HSV 범위
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])

            mask_white = cv2.inRange(hsv, lower_white, upper_white)
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

            img = cv2.bitwise_or(mask_white, mask_yellow)

            xx = 20  
            image = cv_image_raw.copy()  

            
            for i in range(15): # line 
                yy = 460 - (i + 1) * 9  # 9간격으로 위로 이동
                for j in range(32):  # 최대로 가로 개수를 늘림
                    area = img[yy:yy+5, j*20:(j+1)*20]
                    white_pixels = cv2.countNonZero(area)
                    yellow_pixels = cv2.countNonZero(mask_yellow[yy:yy+10, j*20:(j+1)*20]) 

                    if white_pixels > 30:  
                        image[yy:yy+5, j*20:(j+1)*20] = [0, 0, 255]  # 빨간색
                        print(white_pixels)
                    elif yellow_pixels > 80: 
                        image[yy:yy+5, j*20:(j+1)*20] = [0, 0, 255]  # red (노란색 차선)


            mask = cv2.inRange(hsv, (0, 0, 180), (255, 255, 255))
            edge = cv2.Canny(blur, 100, 200, 5)

            cv2.imshow('filter', mask)
            cv2.imshow('Video', image)
            cv2.imshow('Canny', edge)

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