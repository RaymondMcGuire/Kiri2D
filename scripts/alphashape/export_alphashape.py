'''
Author: Xu.WANG
Date: 2021-03-24 23:23:19
LastEditTime: 2021-09-14 19:12:38
LastEditors: Xu.WANG
Description:
FilePath: \Kiri2D\scripts\alphashape\export_alphashape.py
'''

import sys
import numpy as np
from descartes import PolygonPatch
import matplotlib.pyplot as plt
import alphashape
from scipy.spatial import ConvexHull, convex_hull_plot_2d

points = []

path = "D:/project/Kiri/export/xy/cheburashka.xy"
with open(path) as f:
    l_strip = [s.strip() for s in f.readlines()]
    for l in l_strip:
        if " " in l:
            p2 = l.split(" ")
            points.append([float(p2[0]), float(p2[2])])

alpha_shape = alphashape.alphashape(points, 70.)

x, y = alpha_shape.exterior.coords.xy
num = len(x)
out_list = []
for i in range(num):
    out_list.append(str(x[i])+" "+str(y[i]))
    print("clippoints["+str(i)+"].x =" + str(x[i])+";")
    print("clippoints["+str(i)+"].y =" + str(y[i])+";")

path_w = "./cheburashka.xy"
with open(path_w, mode='w') as f:
    f.write(str(num)+"\n")
    f.write('\n'.join(out_list))
