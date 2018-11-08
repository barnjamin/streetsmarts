#! /usr/bin/env python

# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/Tutorial/Advanced/colored_pointcloud_registration.py

import numpy as np
import copy
from open3d import *
import sys



class Registration():

    def __init__(self, path):
        self.img_path = path 
        self.intrinsic = read_pinhole_camera_intrinsic(self.img_path + "/intrinsic.json")
        self.max_nn = 30

    def get_pcd_from_rgbd(self, idx):
        color = read_image("{}/color/{}.jpg".format(self.img_path, idx))
        depth = read_image("{}/depth/{}.png".format(self.img_path, idx))

        rgbd_image = create_rgbd_image_from_color_and_depth(color, depth,  
                        convert_rgb_to_intensity=False, depth_trunc=10.)

        return create_point_cloud_from_rgbd_image(rgbd_image, self.intrinsic)

    def register(self, source, target):

        # colored pointcloud registration
        # This is implementation of following paper
        # J. Park, Q.-Y. Zhou, V. Koltun,
        # Colored Point Cloud Registration Revisited, ICCV 2017

        steps = [{'i':50, 'r':0.04}, {'i':30, 'r':0.02}, {'i':14, 'r':0.01}]

        current_transformation = np.identity(4)

        for s in steps:
            max_iter    = s['i'] 
            radius      = s['r']

            kdtree_opts = KDTreeSearchParamHybrid(radius = radius * 2, max_nn = self.max_nn)

            icp_conv_opts = ICPConvergenceCriteria(relative_fitness = 1e-6, relative_rmse = 1e-6, max_iteration = max_iter)

            source_down = voxel_down_sample(source, radius)
            target_down = voxel_down_sample(target, radius)

            estimate_normals(source_down, kdtree_opts)
            estimate_normals(target_down, kdtree_opts)

            result_icp = registration_colored_icp(source_down, target_down, radius, current_transformation, icp_conv_opts) 

            current_transformation = result_icp.transformation

        return current_transformation 

if __name__ == "__main__":


    reg = Registration("/home/ben/streetsmarts/python/latest")


    start = 10
    source = reg.get_pcd_from_rgbd(start)
    target = reg.get_pcd_from_rgbd(start+1)

    transforms = []
    for x in range(10):
        transforms.append(reg.register(source, target))
        source = copy.deepcopy(target)
        target = reg.get_pcd_from_rgbd(x+start+2)

    print(transforms)

