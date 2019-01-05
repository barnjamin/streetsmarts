#pragma once
#include <Eigen/Geometry>
#include <vector>

#define betaDef		0.05f		// 2 * proportional gain


double invSqrt(double x);

class Pose {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;

    //Vector of positions
    std::vector<Eigen::Vector3d> path;

    //Vector of orientations
    std::vector<Eigen::Vector3d> looking_at;


    //Time between frames
    double time_delta;

    //Last time GetTransform was called, used to compute transform between calls
    int last_check_idx;

    float fps;
    float beta = betaDef;
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

    //Current orientation of the device
    Eigen::Quaterniond orientation;

public:
    //Initialize vectors
    Pose(int fps);    

    //Get the transform from the last check to now
    Eigen::Matrix4d GetTransform();

    //Add the latest accel/gyro readings to pose
    void Update(std::vector<double> accel, std::vector<double> gyro);

    //Used when RGBDOdometry disagrees
    void SetOrientation(Eigen::Quaterniond orientation);

    //Used for gl display
    Eigen::Quaterniond GetOrientation() { return orientation; }

    void MadgwickUpdate(double gx, double gy, double gz, double ax, double ay, double az);

    virtual ~Pose();
};
