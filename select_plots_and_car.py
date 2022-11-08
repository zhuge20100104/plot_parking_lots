# encoding: utf-8
import sys
import datetime
import math
import matplotlib.pyplot as plt
import random
from utils.funcs import *
from collections import namedtuple

ParkingLot = namedtuple("ParkingLot", ["p0", "p1", "p2", "p3"]) 

def print_help():
    print("Please provide the parking lot type you want to select...")
    print("Usage:  python select_plots_and_car.py  {parking_lot_type} {parking_lot_count}")
    print("Supported parking lot types:  [h, v, a]")
    print("h: For horizontal")
    print("v: For vertical")
    print("a: For angle")
    print("parking_lot_count parameter is optional, default count is 3")

def check_plot_type(plot_type):
    if plot_type != 'h' and plot_type != 'v' and plot_type != 'a':
        print("Please select a supported plot type")
        print("[h, v, a]")
        return False
    return True



def show_select_car_mod(ix, iy, plot_type):
    car_mod = list()
    # Car 大小 [4.83, 2]
    if plot_type == 'h': 
        iy_new = iy+2
        ix_new = ix+4.83
        p0 = (ix, iy)
        p1 = (ix, iy_new)
        p2 = (ix_new, iy_new)
        p3 = (ix_new, iy)

    elif plot_type == 'v':
        iy_new = iy+4.83
        ix_new = ix+2
        p0 = (ix, iy)
        p1 = (ix, iy_new)
        p2 = (ix_new, iy_new)
        p3 = (ix_new, iy)
    else:
        # 倾斜车位大小和位置，这个比较复杂，得仔细看
        # 7.99, 4.0
        # 倾斜车位的角度在 30-60度   pi/6 -->  pi/3
        n = random.randint(3, 4)
        angle = math.pi/n
        p0 = (ix, iy)
        p1 = (ix + 4.83*math.cos(angle), iy +  4.83* math.sin(angle))
        p2 = (ix + 4.83*math.cos(angle) + 2, iy +  4.83* math.sin(angle))
        p3 = (ix+ 2, iy)

     
    car_mod.append(p0)
    car_mod.append(p1)
    car_mod.append(p2)
    car_mod.append(p3)

    print("Current car MOD: ")
    print(str(car_mod))

    xy = conv_tuple_list_to_x_y_map(car_mod)
    plt.scatter(xy.x, xy.y, s=POINT_SIZE , cmap=plt.cm.Spectral, alpha=0.8)
    update_x_y_axis_and_replot()


def  show_common_mod(ix, iy, width, height, mod_key) :
    iy_new = iy+height
    ix_new = ix+width
    p0 = (ix, iy)
    p1 = (ix, iy_new)
    p2 = (ix_new, iy_new)
    p3 = (ix_new, iy)

    common_mod = list()

    common_mod.append(p0)
    common_mod.append(p1)
    common_mod.append(p2)
    common_mod.append(p3)
    mod_type = MOD_NAME_MAP[mod_key]
    print("Current "+ mod_type + " MOD: ")
    print(str(common_mod))

    xy = conv_tuple_list_to_x_y_map(common_mod)
    plt.scatter(xy.x, xy.y, s=POINT_SIZE , cmap=plt.cm.Spectral, alpha=0.8)
    update_x_y_axis_and_replot()


def show_select_park_lots(ix, iy, plot_type):
    origin_point = (0, 0)
    parking_lots_selected = list()
    
    # Notice that this is an empty parking lots list  
    parking_lots_and_theta_list = list()
    if plot_type == 'h':
        #  水平车位大小 2.4 , 6  
        iy_new = iy+2.4
        ix_new = ix+6

        tan_x = ix + 6/2
        tan_y = iy
        p0 =   (ix_new, iy)
        p1 =   (ix_new, iy_new)
        p2 =   (ix, iy_new) 
        p3 =   (ix, iy)

        for i in range(1, PARKING_LOT_COUNTS):
            parking_lots_empty = list()
            p0_1 = (ix +  i*(6+ 0.2)+6, iy)
            p1_1 = (ix +  i*(6+ 0.2)+6, iy_new)
            p2_1 = (ix +  i*(6 +0.2), iy_new) 
            p3_1 = (ix + i*(6 +0.2), iy)
            parking_lots_empty.append(p0_1)
            parking_lots_empty.append(p1_1)
            parking_lots_empty.append(p2_1)
            parking_lots_empty.append(p3_1)

            tan_x_1 = ix + i*(6 + 0.2) + 6/2
            tan_y_1 = iy
            thetas_empty = calculate_theta_and_opposite_theta(tan_x_1, tan_y_1)
            pat = ParkingLotsAndTheta(parking_lots_empty, thetas_empty)
            parking_lots_and_theta_list.append(pat)

    elif plot_type == 'v':
        # 垂直车位大小  5.4, 2.4
        iy_new = iy+5.4
        ix_new = ix+2.4

        tan_x = ix + 2.4/2
        tan_y = iy
        p0 =  (ix_new, iy)
        p1 =  (ix_new, iy_new)
        p2 =  (ix, iy_new)
        p3 =  (ix, iy)


        for i in range(1, PARKING_LOT_COUNTS):
            parking_lots_empty = list()
            p0_1 =  (ix +  i*(2.4+ 0.2)+2.4, iy)
            p1_1 = (ix +  i*(2.4+ 0.2)+2.4, iy_new)
            p2_1 = (ix +  i*(2.4 +0.2), iy_new) 
            p3_1 =  (ix + i*(2.4 +0.2), iy)    
     
            parking_lots_empty.append(p0_1)
            parking_lots_empty.append(p1_1)
            parking_lots_empty.append(p2_1)
            parking_lots_empty.append(p3_1)

            tan_x_1 = ix + i*(2.4 + 0.2) + 2.4/2
            tan_y_1 = iy
            thetas_empty = calculate_theta_and_opposite_theta(tan_x_1, tan_y_1)
            pat = ParkingLotsAndTheta(parking_lots_empty, thetas_empty)
            parking_lots_and_theta_list.append(pat)
    else:
        # 倾斜车位大小和位置，这个比较复杂，得仔细看
        # 7.99, 4.0
        # 倾斜车位的角度在 30-60度   pi/6 -->  pi/3
        n = random.randint(3, 4)
        angle = math.pi/n
        tan_x =  ix + 4/2
        tan_y =  iy

        p0 = (ix, iy)
        p1 = (ix + 8*math.cos(angle), iy +  8* math.sin(angle))
        p2 = (ix + 8*math.cos(angle) + 4, iy +  8* math.sin(angle))
        p3 = (ix+ 4, iy)

        
        for i in range(1, PARKING_LOT_COUNTS):
            parking_lots_empty = list()
            p0_1 =  (ix + i*(4 +0.2/math.sin(angle)) + 4, iy)
            p1_1 =  (ix + i*(4 +0.2/math.sin(angle)) +  8*math.cos(angle) + 4,  iy +  8* math.sin(angle))
            p2_1 = (ix + i*(4 +0.2/math.sin(angle)) + 8*math.cos(angle), iy +  8* math.sin(angle)) 
            p3_1 =  (ix + i*(4 +0.2/math.sin(angle)), iy) 
            parking_lots_empty.append(p0_1)
            parking_lots_empty.append(p1_1)
            parking_lots_empty.append(p2_1)
            parking_lots_empty.append(p3_1)

            tan_x_1 = ix + i*(4 +0.2/math.sin(angle)) + 4/2
            tan_y_1 = iy
            thetas_empty = calculate_theta_and_opposite_theta(tan_x_1, tan_y_1)
            pat = ParkingLotsAndTheta(parking_lots_empty, thetas_empty)
            parking_lots_and_theta_list.append(pat)
     
    parking_lots_selected.append(p0)
    parking_lots_selected.append(p1)
    parking_lots_selected.append(p2)
    parking_lots_selected.append(p3)
    parking_lots_selected.append(origin_point)
    print("Current selected parking lot: ")
    print(str(parking_lots_selected))

    thetas = calculate_theta_and_opposite_theta(tan_x, tan_y)
    print("Current two parking thetas: ")
    print( thetas.theta)
    print(thetas.opposite_theta)


    print("================Start printing empty parking lots list=============================")
    for pat in parking_lots_and_theta_list:
        print("One==================")
        parking_lots_emp = pat.parking_lot
        thetas_emp = pat.theta

        parking_lots_selected.extend(parking_lots_emp)
        print("Current empty parking lots: ")
        print(str(parking_lots_emp))
        print("Current two empty thetas: ")
        print(thetas_emp.theta)
        print( thetas_emp.opposite_theta)
        print("End One==================")
        print("")
        print("")
    print("================End printing empty parking lots list=============================")

    xy = conv_tuple_list_to_x_y_map(parking_lots_selected)
    # 清除之前的绘制
    plt.clf()
    plt.scatter(xy.x, xy.y, s=POINT_SIZE , cmap=plt.cm.Spectral, alpha=0.8)
    update_x_y_axis_and_replot()



def show_select_uss_data_point(ix, iy,  plot_type):
    near_far_points_selected = list()
    if plot_type == 'h':
        p0 = (ix, iy)
        p1 = (ix+0.4, iy )
    elif plot_type == 'v':
        p0 = (ix, iy)
        p1 = (ix, iy+0.4)
    elif plot_type == 'a':
        p0 = (ix, iy)
        p1 = (ix + 0.2, iy + 0.2)
    near_far_points_selected.append(p0)
    near_far_points_selected.append(p1)

    xy = conv_tuple_list_to_x_y_map(near_far_points_selected)
    print("Current selected uss data points:")

    # print(str(near_far_points_selected))
    for p in near_far_points_selected:
        distance = math.sqrt(p[0] * p[0] + p[1] * p[1])
        print("  ----- " + str(distance)) 
    plt.scatter(xy.x, xy.y, s=POINT_SIZE, cmap=plt.cm.Spectral, marker='x', alpha=0.8)

    update_x_y_axis_and_replot()


def select_parking_lot(plot_type):
  
    fig = plt.figure(figsize=(12.8, 7.68))
    plt.xlim((-20.0, 20.0))
    plt.ylim((-20.0, 20.0))
    plt.scatter(0.0, 0.0 , s=POINT_SIZE, cmap=plt.cm.Spectral, marker='x', alpha=0.8)
    
    def on_select_parking_lot(event):
        ix, iy = event.xdata, event.ydata
        show_select_park_lots(ix, iy, plot_type)
        
    def on_select_uss_data_point_or_mod(event):
        if event.key == 'u':
            # Start to select USS data points
            ix, iy = event.xdata, event.ydata
            show_select_uss_data_point(ix, iy, plot_type)
        elif event.key == 'c':
            # Start to select Car MOD
            ix, iy = event.xdata, event.ydata
            show_select_car_mod(ix, iy, plot_type)
        elif event.key == 'p':
            # Start to select pilliar
            ix, iy = event.xdata, event.ydata
            show_common_mod(ix, iy, 1.0, 1.1, 'p') 
        elif event.key == 'o':
            # Start to select Cone MOD
            ix, iy = event.xdata, event.ydata
            show_common_mod(ix, iy, 0.7, 0.38, 'o')  
        elif event.key == 'r':
            # Start to select person MOD
            ix, iy = event.xdata, event.ydata
            show_common_mod(ix, iy, 0.885, 0.44, 'r')  
        elif event.key == 'b':
            ix, iy = event.xdata, event.ydata
            show_common_mod(ix, iy, 1.54, 0.76, 'b')  


    fig.canvas.mpl_connect("button_press_event", on_select_parking_lot)
    fig.canvas.mpl_connect("key_press_event", on_select_uss_data_point_or_mod)
    print(plot_type)
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_help()
        exit(-1)
    # 使用全局变量来传 PARKING_LOT_COUNTS 这个参数
    global PARKING_LOT_COUNTS
    plot_type = sys.argv[1]
    if len(sys.argv) == 3:
        PARKING_LOT_COUNTS = int(sys.argv[2])
    
    if not check_plot_type(plot_type):
        exit(-1)
    
    # 初始化随机数种子，用当时时间戳初始化，让每次 生成角度的随机数序列都不一样
    random.seed(datetime.datetime.now())
    select_parking_lot(plot_type)
    
    