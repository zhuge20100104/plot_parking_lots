# encoding: utf-8
import matplotlib.pyplot as plt
import math

from utils.funcs import *

# 倾斜停车位尺寸: [7.984985911070852, 4.0, 7.984985911070852, 4.0]
# 倾斜停车位
# parking_slots_points = [(-3.0, -3.0), (-6.0, -10.4), (-2.0, -10.4), (1, -3.0), (0, 0)]


# kCar大小 [4.83, 2]
# parking_slots_points = [(11.3220129, 1.53857648), (16.1347218, 1.81862056), (16.0182209, 3.82072258), (11.205514, 3.5406785)]



# kPillar大小 [1, 1.1]
# parking_slots_points = [(10.9944611, -2.76017785), ( 12.0510464, -2.57636), (11.862174, -1.49072111), (10.8055887, -1.67453873)]

# kCone大小 [0.7, 0.38]
# parking_slots_points = [(7.06837225, -2.68391371), (7.39001274, -3.30202675), (7.73057842, -3.12481022), (7.40893793, -2.50669718)]

# kPerson大小 [0.885, 0.44]
# parking_slots_points = [(3.75767684, 6.61588812), (3.52684903, 5.76174212), (3.95110202, 5.64709044), (4.18193, 6.50123644)]

parking_slots_points = [(-0.10806451612903167, 2.153679653679653), (-0.10806451612903167, 4.553679653679653), (-6.108064516129032, 4.553679653679653), (-6.108064516129032, 2.153679653679653)]
'''
"p0": {
    "x": -1.2,
    "y": -2.5
},
"p1": {
    "x": -1.2,
    "y": -7.9
},
"p2": {
    "x": 1.2,
    "y": -7.9
},
"p3": {
    "x": 1.2,
    "y": -2.5
},
'''


# kBike大小  [1.54, 0.76]
# parking_slots_points = [(-4.84618044, -4.16001511), (-4.81455851, -5.70356035), (-4.05416822, -5.68798304), (-4.08579, -4.14443779)]

'''
	"points": [{
			"x": -4.84618044,
			"y": -4.16001511
		}, {
			"x": -4.81455851,
			"y": -5.70356035
		}, {
			"x": -4.05416822,
			"y": -5.68798304
		}, {
			"x": -4.08579,
			"y": -4.14443779
		}],
'''





# 平行车位尺寸: [2.40000034, 6.000000000000077, 2.3999998500000004, 6.000000000000019]
# 垂直车位尺寸: [5.4, 2.4, 5.4, 2.4]， 这个Case有问题，PlanCmd的车位，必须是ParkingSpace_Fusioned列表的子车位
# 横向车位， 平行停车
# parking_slots_points = [( -3 , -2.92000031), (-3 , -5.32000065), (3, -5.31999969), (3 , -2.91999984), \
#         (-1.2, -2.5), (-1.2, -7.9),(1.2, -7.9),(1.2, -2.5), (0, 0)]


# 垂直车位尺寸: [5.4, 2.4, 5.4, 2.4]
# # 垂直车位头进
# parking_slots_points = [(-1.2, -2.5), (-1.2 , -7.9), (1.2, -7.9), (1.2, -2.5), (0, 0)]

def cal_distance_for_a_park_lot(points):
    distances = list()
    
    cal_idexes = [(0, 1), (1, 2), (2,3), (3, 0)]
    for cal_idx in cal_idexes:
        distance = math.pow(points[cal_idx[0]] [0] - points[cal_idx[1]] [0], 2)  \
            + math.pow(points[cal_idx[0]] [1] - points[cal_idx[1]] [1], 2)
        distance = math.sqrt(distance)
        distances.append(distance)
    
    print("Distances: ")
    print(str(distances))

def cal_distance(points):
    size = len(points) / 4
    for i in range(0, size):
        cal_distance_for_a_park_lot(points[i*4: (i+1)*4])


def show_parking_lots():
    xy = conv_tuple_list_to_x_y_map(parking_slots_points)
    plt.scatter(xy.x, xy.y, s=POINT_SIZE , cmap=plt.cm.Spectral, alpha=0.8)
    plt.xlim((-12.0, 12.0))
    plt.ylim((-12.0, 12.0))    
    plt.xticks(range(-12, 12))
    plt.yticks(range(-12, 12))
    plt.show()


if __name__ == '__main__':
    # 计算车位的长宽
    cal_distance(parking_slots_points)
    # 绘制车位和小车所在位置    
    # (0, 0) 是小车所在位置
    show_parking_lots()
