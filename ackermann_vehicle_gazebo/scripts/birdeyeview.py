import numpy as np
import cv2

class BEV():
    def __init__(self):
        self.points = []

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"마우스 좌클릭: ({x}, {y})")
            self.points.append([x, y])
            #print(self.points)
            #print(len(self.points))

    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype = "float32")
        s = pts.sum(axis = 1)
        rect[0] = pts[0]
        rect[2] = pts[2]
        diff = np.diff(pts, axis = 1)
        rect[1] = pts[1]
        rect[3] = pts[3]
        
        return rect

    def four_point_transform(self, image, pts):
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect
        #print(rect)
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))
        
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")

        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        warped = cv2.resize(warped, None, fx=1, fy=2, interpolation=cv2.INTER_LINEAR)

        return warped

    def birdeyeview(self, image, A):
        #while len(self.points) == 3:
        #    cv2.setMouseCallback("Video", self.mouse_callback, image)
        pts = np.array(A)
        warped = self.four_point_transform(image, pts)
        
        return warped


