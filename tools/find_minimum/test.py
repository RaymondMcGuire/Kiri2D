'''
Author: Xu.WANG raymondmgwx@gmail.com
Date: 2022-09-02 15:32:12
LastEditors: Xu.WANG raymondmgwx@gmail.com
LastEditTime: 2022-09-02 15:38:04
FilePath: \Kiri2D\tools\find_minimum\test.py
Description: 
Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved. 
'''
import matplotlib.pyplot as plt
import numpy as np


def gen_data(N: int) -> np.ndarray:
    return np.random.rand(N, 2)


def sign(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> bool:
    return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])


def point_in_triangle(pt: np.ndarray, tri_pts: np.ndarray) -> bool:
    d1 = sign(pt, tri_pts[0, :], tri_pts[1, :])
    d2 = sign(pt, tri_pts[1, :], tri_pts[2, :])
    d3 = sign(pt, tri_pts[2, :], tri_pts[0, :])

    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

    return not (has_neg and has_pos)


def heron_formula(pts: np.ndarray) -> float:
    a = np.sqrt((pts[0, 0]-pts[1, 0])**2 + (pts[0, 1]-pts[1, 1])**2)
    b = np.sqrt((pts[1, 0]-pts[2, 0])**2 + (pts[1, 1]-pts[2, 1])**2)
    c = np.sqrt((pts[2, 0]-pts[0, 0])**2 + (pts[2, 1]-pts[0, 1])**2)
    s = (a + b + c) / 2
    return np.sqrt(s*(s-a)*(s-b)*(s-c))


def min_dist(find: np.ndarray, pts: np.ndarray) -> float:
    a = np.sqrt((pts[0, 0]-find[0])**2 + (pts[0, 1]-find[1])**2)
    b = np.sqrt((pts[1, 0]-find[0])**2 + (pts[1, 1]-find[1])**2)
    c = np.sqrt((pts[2, 0]-find[0])**2 + (pts[2, 1]-find[1])**2)
    s = (a + b + c)
    return s


def smallest_triangle(pt_to_find: np.ndarray, pts: np.ndarray) -> np.ndarray:
    min_area_tri = np.inf
    tri_pts = None
    for i in range(len(pts)):
        for j in range(i+1, len(pts)):
            for k in range(j+1, len(pts)):
                # if point_in_triangle(pt_to_find, pts[(i, j, k), :]):
                area = min_dist(pt_to_find, pts[(i, j, k), :])
                if area < min_area_tri:
                    min_area_tri = area
                    tri_pts = pts[(i, j, k), :].copy()
    if tri_pts is None:
        raise Exception("No containing triangle found!")
    else:
        return tri_pts


pts = gen_data(100)
pt_to_find = gen_data(1)[0, :]
smallest = smallest_triangle(pt_to_find, pts)
print("Best area", heron_formula(smallest))

# Add first point to the end for drawing purposes
smallest = np.vstack((smallest, smallest[0, :]))

plt.scatter(pts[:, 0], pts[:, 1])
plt.scatter(pt_to_find[0], pt_to_find[1], c="red", s=20)
plt.plot(smallest[:, 0], smallest[:, 1], '-')
plt.show()
