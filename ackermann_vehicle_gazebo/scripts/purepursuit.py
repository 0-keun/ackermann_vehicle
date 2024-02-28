import math
import numpy as np

point_scale = 10

def get_theta(coef):

    y_from_baselink = 0.4 * ((coef[0] * (430 - (5 + 1) * point_scale) + coef[1]) - 255)
    x_from_baselink = 0.75
    theta = math.atan(y_from_baselink/x_from_baselink)
    theta = -theta * 0.1

    return theta


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

    for i in range(y_repeat): # line each 
        yy = 430 - (i + 1) * point_scale

    return coef
