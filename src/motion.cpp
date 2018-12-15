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

// Clears the window and draws the torus.
void display() {
  
    std::lock_guard<std::mutex> lock(mutex);

    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
    gluLookAt(4, 6, 5, 0, 0, 0, 0, 1, 0);

    //glRotatef(q.w(), q.x(), q.y(), q.z());
    auto e = q.toRotationMatrix().eulerAngles(0,1,2);

    std::cout << e << std::endl;
    std::cout << e[0] << std::endl;
    std::cout << e[1] << std::endl;
    std::cout << e[2] << std::endl;

    glRotatef(e[0]*180/M_PI, 1, 0, 0);
    glRotatef(e[1]*180/M_PI, 0, 1, 0);
    glRotatef(e[2]*180/M_PI, 0, 0, 1);

    glColor3f(1.0, 1.0, 1.0);
    glutWireTorus(0.5, 3, 15, 30);

    //std::cout << "Quat: " << q.w() << " "<< q.x() << " "<< q.y() << " " << q.z() << std::endl;
    std::cout << "Quat: " << q0 << " "<< q1 << " "<< q2 << " " << q3 << std::endl;

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
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowPosition(80, 80);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Orientation");
    glutDisplayFunc(display);
    init();
    glutTimerFunc(100, timer, 0);

    // Define frame callback
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    auto callback = [&](const rs2::frame& f) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!f.as<rs2::frameset>()) {
            // Stream that bypass synchronization (such as IMU) will produce single frames
            if(f.get_profile().stream_type() == RS2_STREAM_ACCEL){
                accel = f.as<rs2::motion_frame>().get_motion_data();
                accel.x /= 9.86;
                accel.y /= -9.86;
                accel.z /= 9.86;
            }else if (f.get_profile().stream_type() == RS2_STREAM_GYRO) {
                gyro = f.as<rs2::motion_frame>().get_motion_data();
            }
        }

        //std::cout << "Gyro: " << gyro.x << " "<< gyro.y << " "<< gyro.z << std::endl;
        //std::cout << "Accel: " << accel.x << " "<< accel.y << " "<< accel.z << std::endl;

	MadgwickAHRSupdateIMU(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);

        q = Eigen::Quaternionf(q0, q1, q2, q3);
        //Eigen::Quaternionf qt(q0, q1, q2, q3);
        //q *= qt;
    };

    // Declare RealSense pipeline, encapsulating the actual device and sensors.
    rs2::pipeline pipe;

    // Start streaming through the callback with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    rs2::pipeline_profile profiles = pipe.start(callback);

    //rs2::pipeline pipe;
    //rs2::config cfg;
    //cfg.enable_stream(RS2_STREAM_ACCEL);
    //cfg.enable_stream(RS2_STREAM_GYRO);
    //rs2::pipeline_profile profiles = pipe.start(cfg);

    glutMainLoop();

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


