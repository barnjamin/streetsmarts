#include <iostream>
#include <Eigen/Geometry>
#include <vector>
#include <Core/Core.h>
#include <Registration/PoseGraph.h>
#include "pose.h"
#include <math.h>

const Eigen::Vector3d gravity(0, -9.806, 0);

//Default to 30fps
Pose::Pose(): Pose(30) { }

Pose::Pose(int frames_per_sec) {
    //Initialize pose
    fps = float(frames_per_sec);
    time_delta = 1.0/fps;
    orientation = Eigen::Quaterniond(q0, q1, q2, q3);
    last_check_idx = 0;

    pg.nodes_.push_back(open3d::PoseGraphNode(Eigen::Matrix4d::Identity()));
}

void Pose::Reset(){
    pos = Eigen::Vector3d(0,0,0);
    vel = Eigen::Vector3d(0,0,0);

    path = std::vector<Eigen::Vector3d>();
    orientations = std::vector<Eigen::Quaterniond>();

    pg = open3d::PoseGraph();
}

std::tuple<double, double, double> Pose::Difference(Eigen::Matrix4d odom){

    if(path.size()== 0) {
        return std::make_tuple(0.0, 0.0, 0.0);
    }
    Eigen::Transform<double, 3, Eigen::Affine> o_trans(odom);

    //Get difference of rotation elements in the world
    Eigen::Quaterniond odom_rotation(o_trans.rotation());
    Eigen::Quaterniond imu_rotation = (orientation * orientations[last_check_idx].inverse());
    Eigen::Quaterniond or_diff = imu_rotation * odom_rotation.inverse();
    Eigen::Vector3d qmag = or_diff.norm() * Eigen::Vector3d(1,1,1);
    double qdiff = abs(log(qmag.dot(qmag)/3.0));
     
    //Get difference of translation elements
    Eigen::Vector3d odom_translation(o_trans.translation());
    Eigen::Vector3d imu_translation(pos - path[last_check_idx]);
    Eigen::Vector3d dist = imu_translation - odom_translation;
    double tdiff = dist.dot(dist);

    //Get difference of velocity 
    Eigen::Vector3d veldiff = vel - (odom_translation / time_delta);
    double vdiff = veldiff.dot(veldiff);

    return std::make_tuple(qdiff, tdiff, vdiff);
}

Eigen::Matrix4d Pose::GetTransform() {
    if(path.size()== 0) {
        return Eigen::Matrix4d::Identity();
    }
    // Find the translation between last check and current 
    Eigen::Translation3d t_diff(pos - path[last_check_idx]);

    //std::cout << t_diff.x() << " " << t_diff.y() << " "<< t_diff.z() << std::endl;

    // Find the orientation difference between last check and current
    Eigen::Quaterniond o_diff = (orientation * orientations[last_check_idx].inverse());

    // Combine to Create Transform
    Eigen::Transform<double, 3, Eigen::Affine> t =  t_diff   * o_diff.normalized().toRotationMatrix();

    // Update last check idx
    last_check_idx = path.size() - 1;

    return t.matrix();
}

void Pose::PrintState(){
    std::cout << "Vel: " << vel << std::endl;
    std::cout << "Pos: " << pos << std::endl;
    std::cout << "Quat: " << orientation.w() << " " << orientation.x() << " " << orientation.y() << " "<< orientation.z() << std::endl;
}
void Pose::Update(std::vector<double> accel, std::vector<double> gyro, double timestamp) {
    // Update quaternion through Madgwick filter
    if(last_timestamp != 0){
        auto delta = timestamp - last_timestamp;

        if (delta == 0) {
            return;
        }

        time_delta = delta;
    }

    last_timestamp = timestamp;

    madgwickUpdate(gyro.at(0), gyro.at(1), gyro.at(2), accel.at(0), accel.at(1), accel.at(2));

    //Set Orientation obj from quat vals
    orientation = Eigen::Quaterniond(q0, q1, q2, q3);

    // Compute world accel from orientation && gravity
    auto rot = orientation.normalized().toRotationMatrix();
    auto accel_raw = Eigen::Vector3d (accel.at(0), accel.at(1), accel.at(2));
    auto accel_rot = rot * accel_raw;
    auto world_accel = accel_rot - gravity;

    // Compute position from velocity
    pos[0] = pos[0] + (vel[0] * time_delta) + (world_accel[0] * (time_delta*time_delta))/2;    
    pos[1] = pos[1] + (vel[1] * time_delta) + (world_accel[1] * (time_delta*time_delta))/2;    
    pos[2] = pos[2] + (vel[2] * time_delta) + (world_accel[2] * (time_delta*time_delta))/2;    

    //std::cout << "World accel: " << world_accel << std::cout;
    // Compute Velocity from accel
    vel[0] = vel[0] + (world_accel[0] * time_delta);
    vel[1] = vel[1] + (world_accel[1] * time_delta);
    vel[2] = vel[2] + (world_accel[2] * time_delta);

}

void Pose::Improve(Eigen::Matrix4d dt, Eigen::Matrix4d wt){
    // Add our changes to the posegraph 
    auto s = pg.edges_.size();
    pg.nodes_.push_back(open3d::PoseGraphNode(wt.inverse()));
    pg.edges_.push_back(open3d::PoseGraphEdge(s, s+1));

    //Convert 4x4 matrix to transform
    Eigen::Transform<double, 3, Eigen::Affine> diff(dt);
    Eigen::Transform<double, 3, Eigen::Affine> world(wt);

    //Get rotation/translation elements in the world
    Eigen::Quaterniond rotation(world.rotation());
    Eigen::Translation3d translation(world.translation());

    //Set velocity to translation/time
    Eigen::Translation3d diff_trans(diff.translation());
    Eigen::Vector3d trpos(diff_trans.x(), diff_trans.y(), diff_trans.z());
    //std::cout << trpos << std::endl;
    vel = trpos / time_delta;

    //Add translation to last path element to get current position
    pos = Eigen::Vector3d(translation.x(), translation.y(), translation.z());

    //Add current rotation to last orientation to get current orientation
    orientation = rotation;

    //Reset quaterion values to our current rotation
    q0 = rotation.w();
    q1 = rotation.x();
    q2 = rotation.y();
    q3 = rotation.z();

    // Add our changes to the list
    path.push_back(pos);
    orientations.push_back(orientation);
}

open3d::PoseGraph Pose::GetGraph() {
    //TODO:: add final entry to edges/nodes?
    return pg;
}

// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
void Pose::madgwickUpdate(double gx, double gy, double gz, double ax, double ay, double az) {
	double recipNorm;
	double s0, s1, s2, s3;
	double qDot1, qDot2, qDot3, qDot4;
	double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * time_delta;
	q1 += qDot2 * time_delta;
	q2 += qDot3 * time_delta;
	q3 += qDot4 * time_delta;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

Pose::~Pose(){ }
