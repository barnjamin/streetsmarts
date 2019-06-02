#pragma once
#include <librealsense2/rs.hpp> 
#include <Eigen/Geometry>
#include <vector>
#include <Open3D/Open3D.h>
#include <Registration/PoseGraph.h>

#define betaDef		0.25f // 2 * proportional gain

float invSqrt(float x);

class Pose {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;

    //Pose graph for Open3d stuffs
    open3d::registration::PoseGraph pg;

    //Vector of positions
    std::vector<Eigen::Vector3d> path;

    //Vector of orientations
    std::vector<Eigen::Quaterniond> orientations;


    Eigen::Matrix4d imu_extrinsic;

    //Time between frames
    double time_delta;
    double last_timestamp;

    //Last time GetTransform was called, used to compute transform between calls
    int last_check_idx;

    float fps;
    float beta = betaDef;
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

    //Current orientation of the device
    Eigen::Quaterniond orientation;

    void madgwickUpdate(double gx, double gy, double gz, double ax, double ay, double az);

public:
    //Initialize vectors
    Pose();    
    Pose(int fps);    
    Pose(int fps, Eigen::Matrix4d extrinsic);    
    virtual ~Pose();

    //Get the transform from the last check to now
    Eigen::Matrix4d GetTransform();
    Eigen::Matrix4d GetWorldTransform();

    //Add the latest accel/gyro readings to pose
    void UpdateGyro(rs2_vector data, double timestamp);
    void UpdateAccel(rs2_vector data, double timestamp);

    //Reset the pose estimation
    void Reset();

    //Used for gl display
    Eigen::Quaterniond GetOrientation() { return orientation; }
    Eigen::Vector3d GetPosition() { return pos; }
};
