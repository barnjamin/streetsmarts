#!/bin/bash

$HOME/streetsmarts/build/bin/make_fragments --session $1 --tsdf_cubic 0.25 --tsdf_truncation 0.002 --max_depth 0.3 --voxel_size 0.002
$HOME/streetsmarts/build/bin/refine --session $1 --max_depth 0.4 --voxel_size 0.002 --max_depth_diff 0.5
$HOME/streetsmarts/build/bin/integrate --session $1 --tsdf_cubic 0.5 --tsdf_truncation 0.003 --max_depth 0.3 

