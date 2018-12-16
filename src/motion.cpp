// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include "MadgwickAHRS.h"
#include <Eigen/Geometry>
#include <GL/glut.h>

std::mutex mutex;
rs2_vector accel;
rs2_vector gyro;
Eigen::Quaternionf q(1.0, 0.0, 0.0, 0.0);

Eigen::Vector3f gravity(0, -9.81, 0);

// Clears the window and draws the torus.
void display() {
  
    std::lock_guard<std::mutex> lock(mutex);

    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
    gluLookAt(4, 6, 5, 0, 0, 0, 0, 1, 0);

    auto e = q.toRotationMatrix().eulerAngles(0,1,2);

    glRotatef(e[0]*180/M_PI, 1, 0, 0);
    glRotatef(e[1]*180/M_PI, 0, 1, 0);
    glRotatef(e[2]*180/M_PI, 0, 0, 1);

    glColor3f(1.0, 1.0, 1.0);
    glutWireTorus(0.5, 3, 15, 30);

    // Draw a red x-axis, a green y-axis, and a blue z-axis.  Each of the
    // axes are ten units long.
    glBegin(GL_LINES);
        glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(10, 0, 0);
        glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 10, 0);
        glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 10);
    glEnd();

    glFlush();
    glutSwapBuffers();
}

// Sets up global attributes like clear color and drawing color, and sets up
// the desired projection and modelview matrices.
void init() {
  // Set the current clear color to black and the current drawing color to
  // white.
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glColor3f(1.0, 1.0, 1.0);

  // Set the camera lens to have a 60 degree (vertical) field of view, an
  // aspect ratio of 4/3, and have everything closer than 1 unit to the
  // camera and greater than 40 units distant clipped away.
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, 4.0/3.0, 1, 40);

  // Position camera at (4, 6, 5) looking at (0, 0, 0) with the vector
  // <0, 1, 0> pointing upward.
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(4, 6, 5, 0, 0, 0, 0, 1, 0);
}

void timer(int v) {
  glutPostRedisplay();
  glutTimerFunc(1000/60, timer, v);
}

int main(int argc, char * argv[]) try
{
    //glutInit(&argc, argv);
    //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    //glutInitWindowPosition(80, 80);
    //glutInitWindowSize(800, 600);
    //glutCreateWindow("Orientation");
    //glutDisplayFunc(display);
    //init();
    //glutTimerFunc(100, timer, 0);

    auto callback = [&](const rs2::frame& f) {
        std::lock_guard<std::mutex> lock(mutex);

        if (!f.as<rs2::frameset>()) {
            if(f.get_profile().stream_type() == RS2_STREAM_ACCEL){
                accel = f.as<rs2::motion_frame>().get_motion_data();
            }else if (f.get_profile().stream_type() == RS2_STREAM_GYRO) {
                gyro = f.as<rs2::motion_frame>().get_motion_data();
            }
        }

	MadgwickAHRSupdateIMU(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);

        q = Eigen::Quaternionf(q0, q1, q2, q3);
        q.normalize();

        Eigen::Vector3f avec(accel.x, accel.y, accel.z);
        std::cout << "BeforeRot" << avec << std::endl;

        //auto rot  = q.toRotationMatrix();
        //auto accel_rot = rot * avec;
        //std::cout << "AfterRot " << accel_rot << std::endl;

        //auto subbed_accel = accel_rot - gravity;
        //std::cout << "AfterSub "<< subbed_accel << std::endl;
    
        //auto world_accel = invrot * subbed_accel;
        //std::cout << "AfterReRot"<< world_accel << std::endl;

        auto invrot = q.inverse().toRotationMatrix();
        auto rot_grav = invrot*gravity;
        std::cout << "RotatedGravity"<< rot_grav << std::endl;

        auto sub_avec  = avec - rot_grav;
        std::cout << "SubGrav"<< sub_avec << std::endl;
        
    };

    rs2::pipeline pipe;
    rs2::pipeline_profile profiles = pipe.start(callback);

    while(true){}
    //glutMainLoop();
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


