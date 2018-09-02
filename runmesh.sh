#!/bin/bash
./build/bin/mesh  \
--minz 1.0  --maxz 5.0  \
--voxel-size 0.01 --neighbors 45 \
--mu 2.5 \
--frames 10 \
--fstart 180

pcl_viewer /media/ssd/meshes/*.vtk
