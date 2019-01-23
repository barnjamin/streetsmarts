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

    holy shit im dumb

    need to predict transformation to see odom based on what the camera is looking at not where it is
    same with pose::improve, should math out the movement of the camera based on the transform of what its looking at


    
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
