#!/bin/bash
rm -rf /media/ssd/dons
mkdir /media/ssd/dons

./build/bin/don_seg \
--don_small 0.02 \
--don_large 0.15 \
--don_thresh 0.03 \
--don_rad .02 \
--frames 1 \
--fstart 10 \
--dec-mag 4.0 \
--spat-mag 5.0 \
--spat-a 0.5 \
--temp-a 0.5 \
--temp-d 100


pcl_viewer /media/ssd/dons/don_cluster*
