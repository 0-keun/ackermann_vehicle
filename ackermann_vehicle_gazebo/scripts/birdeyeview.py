# import the necessary packages
import numpy as np
import cv2

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"마우스 좌클릭: ({x}, {y})")

def order_points(pts):

    rect = np.zeros((4, 2), dtype = "float32")
    s = pts.sum(axis = 1)
    rect[0] = pts[0]
    rect[2] = pts[2]
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[1]
    rect[3] = pts[3]
    
    return rect

def four_point_transform(image, pts):
    rect = order_points(pts)
    (tl, tr, br, bl) = rect
    print(rect)
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


def birdeyeview(image, A):
    pts = np.array(A)
    warped = four_point_transform(image, pts)
    # cv2.imshow("Original", image)
    # cv2.imshow("Warped", warped)
    # cv2.setMouseCallback("Original", mouse_callback)
    # cv2.waitKey(0)
    return warped

# A = [(398, 354), (581, 345), (909, 463), (94, 505)]

# image_ = cv2.imread('./slope_test.jpg')
# birdeyeview(image_,A)
