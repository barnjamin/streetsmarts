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
