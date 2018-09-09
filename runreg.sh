#!/bin/bash
#rm -rf /media/ssd/dons
#mkdir /media/ssd/dons

./build/bin/register \
--don_small 0.03 \
--don_large 0.15 \
--don_thresh 0.04 \
--don_rad .03 \
--frames 25 \
--fstart 10 \
--dec-mag 4.0 \
--spat-mag 5.0 \
--spat-a 0.7 \
--temp-a 0.5 \
--temp-d 100 \
--icp_iters 10 \
--icp_leaf 0.05 \
--icp_dist 0.5 


pcl_viewer final.pcd 
