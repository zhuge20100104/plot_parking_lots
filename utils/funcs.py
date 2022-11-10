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

def parking_lot_to_str(parking_lots):
    res_str = '''"p0":{"x": %s,"y": %s },"p1": {"x": %s,"y": %s },"p2":{"x": %s,"y": %s},"p3": {"x": %s,"y": %s}'''
    return res_str % (parking_lots[0][0], parking_lots[0][1], parking_lots[1][0], parking_lots[1][1], \
        parking_lots[2][0], parking_lots[2][1], parking_lots[3][0], parking_lots[3][1])


if __name__ == '__main__':
    parking_lots_ = [(7.983870967741932, 3.303571428571427), (7.983870967741932, 8.703571428571427), (5.583870967741932, 8.703571428571427), (5.583870967741932, 3.303571428571427)]
    res = parking_lot_to_str(parking_lots_)  
    print(parking_lots_)
    print(res)