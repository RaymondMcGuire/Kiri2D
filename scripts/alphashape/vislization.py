'''
Author: Xu.WANG
Date: 2021-03-24 23:23:19
LastEditTime: 2021-09-14 19:18:35
LastEditors: Xu.WANG
Description: 
FilePath: \Kiri2D\scripts\alphashape\vislization.py
'''

import sys
from descartes import PolygonPatch
import matplotlib.pyplot as plt
import alphashape

points = []

path = "D:/project/Kiri/export/xy/cow.xy"
with open(path) as f:
    l_strip = [s.strip() for s in f.readlines()]
    for l in l_strip:
        if " " in l:
            p2 = l.split(" ")
            points.append([float(p2[0]), float(p2[2])])

alpha_shape = alphashape.alphashape(points, 20.)

fig, ax = plt.subplots()
# ax.scatter(*zip(*points))
ax.add_patch(PolygonPatch(alpha_shape, alpha=0.2))
plt.show()
