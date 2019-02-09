##Street Smarts


```
Front End
    Generate Initial Pose Graph and Fragments

Back End
    Optimize PoseGraph to generate submap 
    Update Pose Correction to Improve Estimation
    Incorporate submap to map

Detector
    Find discontinuity of color and normals on road surface 
    Annotate with prediction of impairment type    
```



```
stream.cpp:
    When to save file -- X Seconds, X frames, Some gap or inconsistency

    Handle frames in callback for higher frequency imu handling

    Kick off mesher in its own thread?

combine.cpp:
    Combine Using ICP 

    Improve PoseGraph

    Generate TSDF of whole scan

pose.cpp:
    Add Correction Factors && EKF

    Generate Pose Graph 
    
analyze.cpp:
    Load Point Cloud - Combined or Stream

    Diff of Norms - Faster

    Group Points - Euclidian Clustering

    PointNet - Learn Features of Potholes and Cracks
    
    Annotate Impairments - Locations with Transformation to center of shape (Sphere, Cube) + Type

```




```

mkdir build
cd build
cmake ..
make
./bin/stream --fps 60 --frames 360

Visualizer mesh fragment-0.ply
```
