import math
import numpy as np

'''
[Warning] x, y coordinates in the image and kinematic model are different.
'''

point_scale = 10

def get_delta(coef,L,ld):
    ## Lookahead Distance: x_from_baselink [m] (about 1ea * 0.125m)
    ## Lateral Difference: 0.4 * offset / center_pixel
    x = 6

    x_from_baselink = x * 0.125
    y_from_baselink = 0.4 * ((coef[0] * (430 - x * point_scale) + coef[1]) - 255) / 255
    theta = math.atan(y_from_baselink/x_from_baselink)
    delta = math.atan(2*L*math.sin(theta)/ld)

    return delta

def get_poly(coef, x):
    return coef[0]*x + coef[1]

def get_coef_oneside(y_repeat,lane_pred,xy):
    k = 0

    for i in range (y_repeat):
        if lane_pred[i][0] != 0:
            xy[1][k] = lane_pred[i][0]
            xy[0][k] = lane_pred[i][1]
            k += 1

    coef = np.polyfit(xy[0], xy[1], 1)

    return coef

def estimate_line(y_repeat,left_lane_pred,right_lane_pred,image):
    left_count = np.count_nonzero(left_lane_pred) - y_repeat
    right_count = np.count_nonzero(right_lane_pred) - y_repeat
    coef = [0,0]

    xy_l = np.zeros((2,left_count))
    xy_r = np.zeros((2,right_count))

    if left_count > 2:
        l_coef = get_coef_oneside(y_repeat,left_lane_pred,xy_l,yy)
    if right_count > 2:
        r_coef = get_coef_oneside(y_repeat,right_lane_pred,xy_r,yy)


    if right_count > 2 and left_count > 2:
        coef[0] = (l_coef[0] + r_coef[0])/2
        coef[1] = (l_coef[1] + r_coef[1])/2
        for i in range(y_repeat): 
            yy = 430 - (i + 1) * point_scale
            poly = get_poly(coef,yy)
            tem = int(poly)
            image[yy:yy+5, tem-10:tem] = [0, 255, 0]

    elif right_count > 2 and left_count < 3:
        coef[0] = r_coef[0]
        coef[1] = -(442*r_coef[0]) + 225
        for i in range(y_repeat):
            yy = 430 - (i + 1) * point_scale
            poly = get_poly(coef,yy)
            tem = int(poly)
            image[yy:yy+5, tem-10:tem] = [0, 255, 0]

    elif left_count > 2 and right_count < 3:
        coef[0] = l_coef[0]
        coef[1] = -(442*l_coef[0]) + 225
        for i in range(y_repeat):
            yy = 430 - (i + 1) * point_scale
            poly = get_poly(coef,yy)
            tem = int(poly)
            image[yy:yy+5, tem-10:tem] = [0, 255, 0]

    else:
        coef[0] = 0
        coef[1] = 225

    return coef