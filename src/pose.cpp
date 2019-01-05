#include <iostream>
#include <Eigen/Geometry>
#include <vector>
#include "pose.h"

const Eigen::Vector3d gravity(0, -9.81, 0);

Pose::Pose(){
    int frames_per_sec = 30;
    fps = float(frames_per_sec);
    time_delta = 1.0/fps;
    orientation = Eigen::Quaterniond(q0, q1, q2, q3);
}

Pose::Pose(int frames_per_sec) {
    //Initialize pose
    fps = float(frames_per_sec);
    time_delta = 1.0/fps;
    orientation = Eigen::Quaterniond(q0, q1, q2, q3);
}

Pose::~Pose(){ }

Eigen::Matrix4d Pose::GetTransform() {
    // Check the last time we got a transform
    // Find the tranform between last check and now 
}

void Pose::Update(std::vector<double> accel, std::vector<double> gyro) {
    // Update quaternion through Madgwick filter
    MadgwickUpdate(gyro.at(0), gyro.at(1), gyro.at(2), accel.at(0), accel.at(1), accel.at(2));

    //Set Orientation obj from quat vals
    orientation = Eigen::Quaterniond(q0, q1, q2, q3);

    // Compute accel from orientation && gravity

    auto rot = orientation.normalized().toRotationMatrix();
    auto accel_raw = Eigen::Vector3d (accel.at(0), accel.at(1), accel.at(2));
    auto accel_rot = rot * accel_raw;
    auto world_accel = accel_rot - gravity;

    // Compute Velocity from accel
    vel[0] = vel[0] + world_accel[0] * time_delta;
    vel[1] = vel[1] + world_accel[1] * time_delta;
    vel[2] = vel[2] + world_accel[2] * time_delta;

    // Compute position from velocity
    pos[0] = pos[0] + vel[0] * time_delta + (world_accel[0] * time_delta)/2;    
    pos[1] = pos[1] + vel[1] * time_delta + (world_accel[1] * time_delta)/2;    
    pos[2] = pos[2] + vel[2] * time_delta + (world_accel[2] * time_delta)/2;    


    path.push_back(pos);
    orientations.push_back(orientation);
}

void Pose::SetOrientation(Eigen::Quaterniond){
    //Noop rn
}


void Pose::MadgwickUpdate(double gx, double gy, double gz, double ax, double ay, double az) {
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
	recipNorm = double(invSqrt(float(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)));
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

// Fast inverse square-root See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
