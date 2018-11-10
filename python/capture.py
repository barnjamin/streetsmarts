#! /usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import cv2
import sys 
import datetime
import os
import json


#Create direcotry for new captures
filedate = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
os.mkdir(filedate)
os.mkdir(filedate+"/depth")
os.mkdir(filedate+"/color")

if os.path.islink("latest"):
    os.unlink("latest")

os.symlink(filedate, "latest")

def write_intrinsics(cfg):
    profile = cfg.get_stream(rs.stream.depth)
    intrin = profile.as_video_stream_profile().get_intrinsics()

    print("Intrinsics: {}".format(intrin.width))
    
    intrinsic_ = {
        'width':intrin.width,
        'height':intrin.height,
        'intrinsic_matrix' : [ 
            intrin.fx, 0, 0,
            0, intrin.fy, 0,
            intrin.ppx, intrin.ppy, 1,
        ],
    }

    f = open(filedate+"/intrinsic.json", "w+")
    f.write(json.dumps(intrinsic_))
    f.close()


config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

try:
    pipe = rs.pipeline()
    cfg = pipe.start(config);

    write_intrinsics(cfg)
    
    idx = 0
    while True:
        frames = pipe.wait_for_frames()

        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        if not depth or not color:
            continue

        depth_data = depth.as_frame().get_data()
        depth_image = np.asanyarray(depth_data)
        cv2.imwrite("{}/depth/{}.png".format(filedate, idx), depth_image)

        color_data = color.as_frame().get_data()
        color_image = np.asanyarray(color_data)
        cv2.imwrite("{}/color/{}.jpg".format(filedate, idx), color_image)

        idx += 1

finally:
    pipe.stop()
