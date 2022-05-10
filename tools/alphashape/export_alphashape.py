'''
Author: Xu.WANG
Date: 2021-03-24 23:23:19
LastEditTime: 2021-10-05 23:06:45
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

ext = ".xy"
path = "D:/project/Kiri/export/xy/"
xy_files = ["xyzrgb_dragon","woody","teapot","spot","nefertiti","lucy","horse","homer","beast","alligator"]
xy_params = [75,42,40,40,50,250,50,30,50,200]

def export_alphashape(path,files,params):
    for idx in range(len(files)):
        file_path = path + files[idx] + ext
        points = []
        with open(file_path) as f:
            l_strip = [s.strip() for s in f.readlines()]
            for l in l_strip:
                if " " in l:
                    p2 = l.split(" ")
                    points.append([float(p2[0]), float(p2[2])])

        alpha_shape = alphashape.alphashape(points, params[idx])

        x, y = alpha_shape.exterior.coords.xy
        num = len(x)
        out_list = []
        for i in range(num):
            out_list.append(str(x[i])+" "+str(y[i]))
            # print("clippoints["+str(i)+"].x =" + str(x[i])+";")
            # print("clippoints["+str(i)+"].y =" + str(y[i])+";")

        path_w = "../../resources/alpha_shapes/"+ files[idx] + ext
        with open(path_w, mode='w') as f:
            f.write(str(num)+"\n")
            f.write('\n'.join(out_list))
            print("write file:"+files[idx])

export_alphashape(path,xy_files,xy_params)