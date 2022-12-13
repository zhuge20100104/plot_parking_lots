
import matplotlib.pyplot as plt
import matplotlib
import numpy as np


def plot_lines():
    print("Plot lines")
    pts = [(0, 3), (1, 7), (2, 5), (3, 9)]
    x_list = list()
    y_list = list()
    for pt in pts:
        x_list.append(pt[0])
        y_list.append(pt[1])
    
    for i in range(0, len(pts)/2):
        x_in = x_list[i*2: (i+1)*2]
        y_in = y_list[i*2: (i+1)*2]
        plt.plot(x_in, y_in, color='dimgray', linewidth=0.5, label="line")


def plot_rect():
    print("Plot rectangle")
    # Notice that we should point to (0, 0) at the end
    pts = [(0, 0), (0, 1), (4, 1), (4, 0), (0, 0)]
    x_list = list()
    y_list = list()
    for pt in pts:
        x_list.append(pt[0])
        y_list.append(pt[1])
    plt.plot(x_list, y_list, color='green', linewidth=3.0)
    

# Just add label is ok
def plot_title():
    print("Plot title")


def plot_circle():
    print("Plot circle")
    plt.scatter(2, 2, s=150, facecolors='none', edgecolors='black')

if __name__ == '__main__':
    plt.xlim((-5.0, 12.0))
    plt.ylim((-5.0, 12.0))
    
    plot_lines()
    plot_rect()
    plot_title()
    plot_circle()
    # Label must work with the help of legend
    plt.legend(loc='best')
    plt.show()

