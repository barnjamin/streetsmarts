##Street Smarts


```
TODO:



Physical Params:
    Angle of camera
    D435 vs D415

Test Scripts:
    fps (30/60/90)
    auto-exposure (on/off)
    white balance (on/off)
    filter params (on/off)

Reconstruction:
    Timestamps for images
    Covariance matrix from DOP?
    Loop Closure from distance 

Analysis:


Web:
    Capture Controller
    Point cloud viewer



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
