# encoding: utf-8
import sys
import datetime
import math
import matplotlib.pyplot as plt
import random
from utils.funcs import *


def print_help():
    print("Please provide the parking lot type you want to select...")
    print("Usage:  python select_plots_and_car.py  {parking_lot_type}")
    print("Supported parking lot types:  [h, v, a]")
    print("h: For horizontal")
    print("v: For vertical")
    print("a: For angle")

def check_plot_type(plot_type):
    if plot_type != 'h' and plot_type != 'v' and plot_type != 'a':
        print("Please select a supported plot type")
        print("[h, v, a]")
        return False
    return True


def show_select_park_lots(ix, iy, plot_type):
    origin_point = (0, 0)
    parking_lots_selected = list()
    if plot_type == 'h':
        #  水平车位大小 2.4 , 6  
        iy_new = iy+2.4
        ix_new = ix+6
        p0 = (ix, iy)
        p1 = (ix, iy_new)
        p2 = (ix_new, iy_new)
        p3 = (ix_new, iy)
    elif plot_type == 'v':
        # 垂直车位大小  5.4, 2.4
        iy_new = iy+5.4
        ix_new = ix+2.4
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
        p1 = (ix + 8*math.cos(angle), iy +  8* math.sin(angle))
        p2 = (ix + 8*math.cos(angle) + 4, iy +  8* math.sin(angle))
        p3 = (ix+ 4, iy)
    
    parking_lots_selected.append(p0)
    parking_lots_selected.append(p1)
    parking_lots_selected.append(p2)
    parking_lots_selected.append(p3)
    parking_lots_selected.append(origin_point)
    print("Current selected parking lot: ")
    print(str(parking_lots_selected))
    
    xy = conv_parking_slots_to_x_y_map(parking_lots_selected)
    # 清除之前的绘制
    plt.clf()
    plt.scatter(xy.x, xy.y, s=POINT_SIZE , cmap=plt.cm.Spectral, alpha=0.8)
    plt.xlim((-12.0, 12.0))
    plt.ylim((-12.0, 12.0))    
    plt.xticks(range(-12, 12))
    plt.yticks(range(-12, 12))
    plt.show()

def select_parking_lot(plot_type):
    fig = plt.figure()
    plt.xlim((-12.0, 12.0))
    plt.ylim((-12.0, 12.0))
    
    def on_click_listener(event):
        ix, iy = event.xdata, event.ydata
        show_select_park_lots(ix, iy, plot_type)
        

    fig.canvas.mpl_connect("button_press_event", on_click_listener)
    print(plot_type)
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_help()
        exit(-1)
    plot_type = sys.argv[1]
    
    if not check_plot_type(plot_type):
        exit(-1)
    
    # 初始化随机数种子，用当时时间戳初始化，让每次 生成角度的随机数序列都不一样
    random.seed(datetime.datetime.now())
    select_parking_lot(plot_type)
    
    