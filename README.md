##Street Smarts


```
TODO:


Test Scripts:
    d435i
    d415
    fps
    auto-exposure
    white balance
    height/width
    filter params
    Align params
    

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
