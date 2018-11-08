#! /usr/bin/env python

from open3d import *
import numpy as np


base_path = "/home/ben/Open3D/examples/TestData/"

pcd = read_point_cloud(base_path+"fragment.ply")
#draw_geometries([pcd])

print("Downsample the point cloud with a voxel of 0.05")

downpcd = voxel_down_sample(pcd, voxel_size = 0.02)


estimate_normals(downpcd, search_param = KDTreeSearchParamHybrid(radius = 0.05, max_nn = 30))
small_norm = np.asarray(downpcd.normals).copy()
estimate_normals(downpcd, search_param = KDTreeSearchParamHybrid(radius = 0.15, max_nn = 30))
large_norm = np.asarray(downpcd.normals).copy()

a = large_norm - small_norm

print(a[10::])
print(a.max())
print(a.shape)
