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
        self.depth_dir = os.path.join(self.root_dir, 'depth' , 'sequences', '00')
        self.color_dir = os.path.join(self.root_dir, 'color' , 'sequences', '00')
        self.poses_dir = os.path.join(self.root_dir, 'poses')

    def __len__(self):
        return 0 

    def get_calibration(self):
        calib_filename = os.path.join(self.calib_dir, 'calib.txt')
        return utils.Calibration(calib_filename)

    def get_image(self, idx):
        img_filename = os.path.join(self.image_dir, '%06d.png'%(idx))
        return utils.load_image(img_filename)

    def get_lidar(self, idx): 
        lidar_filename = os.path.join(self.lidar_dir, '%06d.bin'%(idx))
        return utils.load_velo_scan(lidar_filename)

    def get_depth(self, idx):
        img_filename = os.path.join(self.depth_dir, '%06d.png'%(idx))
        return utils.load_image(img_filename)

    def get_color(self, idx):
        img_filename = os.path.join(self.color_dir, '%06d.png'%(idx))
        return utils.load_image(img_filename)


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
    imgfov_pts_2d = pts_2d[fov_inds,:] 
    imgfov_pc_rect = calib.project_velo_to_rect(imgfov_pc_velo)

    import matplotlib.pyplot as plt
    cmap = plt.cm.get_cmap('gray', 256)
    cmap = np.array([cmap(i) for i in range(256)])[:,:3]*255

    depth_img = np.zeros((img_height,img_width, 3), np.uint8)
    for i in range(imgfov_pts_2d.shape[0]):
        depth = imgfov_pc_rect[i,2]
        color = cmap[int(640.0/depth),:]
        
        cv2.circle(depth_img, (int(np.round(imgfov_pts_2d[i,0])), int(np.round(imgfov_pts_2d[i,1]))), 2, color=tuple(color), thickness=-1)

    return depth_img 

if __name__=='__main__':

    root_dir = '/home/ben/kitti'
    dataset = kitti_object(root_dir)
    calib = dataset.get_calibration()

    depth_dir = os.path.join(root_dir, 'depth')
    color_dir = os.path.join(root_dir, 'color')

    if not os.path.exists(depth_dir):
            os.makedirs(depth_dir)

    if not os.path.exists(color_dir):
            os.makedirs(color_dir)

    for data_idx in range(4000):
        print("Working on {}".format(data_idx))
        
        img = dataset.get_image(data_idx)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 

        img_filename = os.path.join(color_dir, '%06d.jpg'%(data_idx))
        cv2.imwrite(img_filename, img)

        pc_velo = dataset.get_lidar(data_idx)[:,0:3]

        img_height, img_width, img_channel = img.shape
        depth_img = show_lidar_on_image(pc_velo, img, calib, img_width, img_height) 

        depth_img_filename = os.path.join(depth_dir, '%06d.png'%(data_idx))
        cv2.imwrite(depth_img_filename, depth_img)


    exit()
