##Street Smarts





```
Front End
    Generate Initial Pose Graph and Fragments
    Pass Fragments/Pose Graph/IMU/GPS to server

Back End
    Optimize PoseGraph to generate submap 
    Update Pose Correction to Improve Estimation
    Incorporate submap to map

Detector
    Find discontinuity of color and normals on road surface 
    Annotate with prediction of impairment type    

```

```
mkdir build
cd build
cmake ..
make -j4
```


```

                 +---------------------+               +----------------------+
                 |                     |               |                      |
                 |       Tx2           |               |       Server         |
          Images |                     |  Fragments    |                      |
            +--->+ Capture Depth/Color |       +------->  Register Fragments  |
+--------+  |    |                     |  And  |       |                      |
|        |  |    | Generate Fragments  |  PoseGraph    |  Refine Registration |
|        |  |    | and PoseGraph       |       |       |                      |
| Camera +--+    |                     |       |       |  Optimize PoseGraph  |
|        |       | Compress Fragment   +-------+       |                      |
|        |       | and Upload to       |               |                      |
|        |       | Server              |  Optimized    |  Send Optimized      |
+--------+       |                     |  PoseGraph----+  PoseGraph to Tx2    |
                 |                     |       |       |                      |
                 | Integrate Images    |       |       |                      |
                 | Using Optimized     +<------+       |                      |
                 | PoseGraph           |               |                      |
                 |                     | Final Model   |                      |
                 | Upload Final Model  +-------------->+ Final Model Analyzed |
                 |                     |               | For Impairments      |
                 +---------------------+               +----------------------+




```
