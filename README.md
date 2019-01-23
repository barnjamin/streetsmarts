##Street Smarts


```

stream.cpp:
    When to save file -- X Seconds, X frames, Some gap or inconsistency

    Kick off mesher in its own thread?

combine.cpp:
    Combine Using ICP 

    Generate PoseGraph

    Generate TSDF of whole scan

    What is PoseGraph Optimizer?


pose.cpp:
    Pose::Improve - use some combination of rgbdodom + imu readings instead of overwriting completely

    
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
