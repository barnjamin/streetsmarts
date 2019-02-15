#! /usr/bin/python

''' Prepare KITTI data for 3D object detection.

Author: Charles R. Qi
Date: September 2017
'''


from __future__ import print_function

import os
import sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)

import utils 

import cv2
from PIL import Image
import numpy as np

try:
    raw_input          # Python 2
except NameError:
    raw_input = input  # Python 3


class kitti_object(object):
    '''Load and parse object data into a usable format.'''
    
    def __init__(self, root_dir):
        '''root_dir contains training and testing folders'''
        self.root_dir = root_dir
        self.image_dir = os.path.join(self.root_dir, 'images', 'sequences', '00', 'image_0')
        self.calib_dir = os.path.join(self.root_dir, 'calib' , 'sequences', '00')
        self.lidar_dir = os.path.join(self.root_dir, 'lidar' , 'sequences', '00', 'velodyne')
        self.poses_dir = os.path.join(self.root_dir, 'poses')

    def __len__(self):
        return 0 

    def get_image(self, idx):
        img_filename = os.path.join(self.image_dir, '%06d.png'%(idx))
        print(img_filename)
        return utils.load_image(img_filename)

    def get_lidar(self, idx): 
        lidar_filename = os.path.join(self.lidar_dir, '%06d.bin'%(idx))
        return utils.load_velo_scan(lidar_filename)

    def get_calibration(self, idx):
        calib_filename = os.path.join(self.calib_dir, 'calib.txt')
        return utils.Calibration(calib_filename)

    def get_depth_map(self, idx):
        pass

    def get_top_down(self, idx):
        pass

def get_lidar_in_image_fov(pc_velo, calib, xmin, ymin, xmax, ymax, clip_distance=2.0):
    ''' Filter lidar points, keep those in image FOV '''
    pts_2d = calib.project_velo_to_image(pc_velo)

    fov_inds = (pts_2d[:,0]<xmax) & (pts_2d[:,0]>=xmin) & (pts_2d[:,1]<ymax) & (pts_2d[:,1]>=ymin)

    fov_inds = fov_inds & (pc_velo[:,0]>clip_distance)

    imgfov_pc_velo = pc_velo[fov_inds,:]

    return imgfov_pc_velo, pts_2d, fov_inds

def show_lidar_on_image(pc_velo, img, calib, img_width, img_height):
    ''' Project LiDAR points to image '''
    imgfov_pc_velo, pts_2d, fov_inds = get_lidar_in_image_fov(pc_velo,
             calib, 0, 0, img_width, img_height)

    print(imgfov_pc_velo) #Pointcloud in fov
    print(pts_2d) #UV 
    print(fov_inds) #Bool idx

    #subset of pc
    imgfov_pts_2d = pts_2d[fov_inds,:] 

    #Project pointcloud points in fov to a rectangle view from calibration
    imgfov_pc_rect = calib.project_velo_to_rect(imgfov_pc_velo)

    import matplotlib.pyplot as plt

    cmap = plt.cm.get_cmap('hsv', 256)

    cmap = np.array([cmap(i) for i in range(256)])[:,:3]*255

    for i in range(imgfov_pts_2d.shape[0]):
        depth = imgfov_pc_rect[i,2]
        color = cmap[int(640.0/depth),:]
        cv2.circle(img, (int(np.round(imgfov_pts_2d[i,0])),
            int(np.round(imgfov_pts_2d[i,1]))),
            2, color=tuple(color), thickness=-1)

    Image.fromarray(img).show() 

    return img

if __name__=='__main__':
    dataset = kitti_object('/home/ben/kitti')

    data_idx = 0

    img = dataset.get_image(data_idx)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 

    pc_velo = dataset.get_lidar(data_idx)[:,0:3]

    calib = dataset.get_calibration(data_idx)

    img_height, img_width, img_channel = img.shape

    show_lidar_on_image(pc_velo, img, calib, img_width, img_height) 

    raw_input()

    exit()
