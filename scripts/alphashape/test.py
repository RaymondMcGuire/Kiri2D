'''
Author: Xu.WANG
Date: 2021-03-24 23:23:19
LastEditTime: 2021-03-24 23:45:31
LastEditors: Xu.WANG
Description: 
FilePath: \Kiri2D\scripts\alphashape\test.py
'''

import sys
from descartes import PolygonPatch
import matplotlib.pyplot as plt
import alphashape

points = [(0., 0.), (0.5, 0.5), (0., 1.), (1., 1.), (1., 0.),
          (0.5, 0.25), (0.5, 0.75), (0.25, 0.5), (0.75, 0.5)]

# example1
# fig, ax = plt.subplots()
# ax.scatter(*zip(*points))
# plt.show()


#alpha_shape = alphashape.alphashape(points, 2.)

# Generate an Alpha Shape by Solving for an Optimal Alpha Value
alpha_shape = alphashape.alphashape(points)

print(alpha_shape)

fig, ax = plt.subplots()
ax.scatter(*zip(*points))
ax.add_patch(PolygonPatch(alpha_shape, alpha=0.2))
plt.show()
