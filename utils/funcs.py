from collections import namedtuple
import matplotlib.pyplot as plt
import math

XYs = namedtuple("XYs", ["x", "y"])
Thetas = namedtuple("Thetas", ["theta", "opposite_theta"])
ParkingLotsAndTheta = namedtuple("ParkingLotsAndTheta", ["parking_lot", "theta"])

POINT_SIZE = 50
PARKING_LOT_COUNTS = 3
MOD_NAME_MAP = {
    'c': "kCar",
    'p': "kPillar",
    'o': "kCone",
    'r': "kPerson",
    'b': "kBike"
}

def conv_tuple_list_to_x_y_map(ps_points):
    x_list = list()
    y_list = list()
    for ps_point in ps_points:
        x_list.append(ps_point[0])
        y_list.append(ps_point[1])
    xys = XYs(x_list, y_list)
    return xys

def update_x_y_axis_and_replot():
    plt.xlim((-20.0, 20.0))
    plt.ylim((-20.0, 20.0))    
    plt.xticks(range(-20, 20))
    plt.yticks(range(-20, 20))
    plt.show()


def calculate_theta_and_opposite_theta(tan_x, tan_y):
    theta = math.atan((float)(tan_y)/tan_x)

    if theta < math.pi:
        opposite_theta = math.pi + theta
    else:
        opposite_theta = theta  - math.pi
    return Thetas(theta, opposite_theta)