#pragma once
#include <Eigen/Geometry>
#include <vector>

#define betaDef		0.05f		// 2 * proportional gain


float invSqrt(float x);

class Pose {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;

    //Vector of positions
    std::vector<Eigen::Vector3d> path;

    //Vector of orientations
    std::vector<Eigen::Quaterniond> orientations;


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

public:
    //Initialize vectors
    Pose();    
    Pose(int fps);    

    //Get the transform from the last check to now
    Eigen::Matrix4d GetTransform();

    //Add the latest accel/gyro readings to pose
    void Update(std::vector<double> accel, std::vector<double> gyro, double timestamp);

    //Improve Current orientation using RGBDOdometry
    void Improve(Eigen::Matrix4d diff, Eigen::Matrix4d world);

    //void Reset();

    //Used for gl display
    Eigen::Quaterniond GetOrientation() { return orientation; }

    void MadgwickUpdate(double gx, double gy, double gz, double ax, double ay, double az);

    virtual ~Pose();
};
