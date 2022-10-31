from collections import namedtuple
XYs = namedtuple("XYs", ["x", "y"])

POINT_SIZE = 50

def conv_parking_slots_to_x_y_map(ps_points):
    x_list = list()
    y_list = list()
    for ps_point in ps_points:
        x_list.append(ps_point[0])
        y_list.append(ps_point[1])
    xys = XYs(x_list, y_list)
    return xys