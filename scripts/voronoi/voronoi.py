'''
Author: Xu.WANG
Date: 2021-04-07 10:51:40
LastEditTime: 2021-04-07 10:57:31
LastEditors: Xu.WANG
Description: 
FilePath: \Kiri2D\scripts\voronoi\voronoi.py
'''
import math
import random
from PIL import Image
from scipy import spatial
import numpy as np

# define the size of the x and y bounds
screen_width = 500
screen_height = 500

# define the number of points that should be used
number_of_points = 500

# randomly generate a list of n points within the given x and y bounds
point_x_coordinates = random.sample(range(0, screen_width), number_of_points)
point_y_coordinates = random.sample(range(0, screen_height), number_of_points)
points = list(zip(point_x_coordinates, point_y_coordinates))

# each point needs to have a corresponding list of pixels
point_pixels = []
for i in range(len(points)):
    point_pixels.append([])

# build a search tree
tree = spatial.KDTree(points)

# build a list of pixed coordinates to query
pixel_coordinates = np.zeros((screen_height*screen_width, 2))
i = 0
for pixel_y_coordinate in range(screen_height):
    for pixel_x_coordinate in range(screen_width):
        pixel_coordinates[i] = np.array(
            [pixel_x_coordinate, pixel_y_coordinate])
        i = i+1

# for each pixel within bounds, determine which point it is closest to and add it to the corresponding list in point_pixels
[distances, indices] = tree.query(pixel_coordinates)

i = 0
for pixel_y_coordinate in range(screen_height):
    for pixel_x_coordinate in range(screen_width):
        point_pixels[indices[i]].append(
            (pixel_x_coordinate, pixel_y_coordinate))
        i = i+1

# each point needs to have a corresponding centroid
point_pixels_centroid = []

for pixel_group in point_pixels:
    x_sum = 0
    y_sum = 0
    for pixel in pixel_group:
        x_sum += pixel[0]
        y_sum += pixel[1]

    x_average = x_sum / max(len(pixel_group), 1)
    y_average = y_sum / max(len(pixel_group), 1)

    point_pixels_centroid.append((round(x_average), round(y_average)))


# display the resulting voronoi diagram
display_voronoi = Image.new("RGB", (screen_width, screen_height), "white")
# for pixel_group in point_pixels:
#     rgb = random.sample(range(0, 255), 3)
#     for pixel in pixel_group:
#         display_voronoi.putpixel( pixel, (rgb[0], rgb[1], rgb[2], 255) )

for centroid in point_pixels_centroid:
    # print(centroid)
    display_voronoi.putpixel(centroid, (1, 1, 1, 255))

# display_voronoi.show()
display_voronoi.save("test.png")
