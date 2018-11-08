import pyrealsense2 as rs
import numpy as np
import cv2
import sys 
import datetime
import os


# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()


#Create direcotry for new captures
filedate = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
os.mkdir(filedate)

try:
    pipe = rs.pipeline()
    pipe.start();

    idx = 0
    while True:
        frames = pipe.wait_for_frames()

        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        # Tell pointcloud object to map to this color frame
        pc.map_to(color)

        # Generate the pointcloud and texture mappings
        points = pc.calculate(depth)

        print("Saving to 1.ply...")
        points.export_to_ply("{}/{}.ply".format(filedate, idx), color)
        print("Done")

        idx += 1

finally:
    pipe.stop()
