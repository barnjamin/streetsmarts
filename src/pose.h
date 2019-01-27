#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <Core/Core.h>
#include <Registration/PoseGraph.h>

#define betaDef		0.25f // 2 * proportional gain

float invSqrt(float x);

class Pose {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;

    //Pose graph for Open3d stuffs
    open3d::PoseGraph pg;

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
    void Update(std::vector<double> accel, std::vector<double> gyro, double timestamp);

    //Improve Current orientation using RGBDOdometry
    void Improve(Eigen::Matrix4d world_transform);

    //Get the difference between imu and odom 
    std::tuple<double, double, double> Difference(Eigen::Matrix4d odom);

    //Reset the pose estimation
    void Reset();

    //
    void PrintState();

    //Return the aggregated pose graph
    open3d::PoseGraph GetGraph();

    //Used for gl display
    Eigen::Quaterniond GetOrientation() { return orientation; }
    Eigen::Vector3d GetPosition() { return pos; }
};
