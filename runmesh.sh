#!/bin/bash
rm -rf /media/ssd/meshes
mkdir /media/ssd/meshes

./build/bin/mesh  \
--minz 1.0  --maxz 5.0  \
--voxel-size 0.1 --neighbors 45 \
--mu 2.5 \
--frames 10 \
--fstart 10 \
--dec-mag 5.0 \
--spat-mag 5.0 \
--spat-a 0.5 \
--temp-a 0.5 \
--temp-d 100

pcl_viewer /media/ssd/meshes/*.vtk
