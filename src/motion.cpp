#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <fstream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include "MadgwickAHRS.h" // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
#include <Eigen/Geometry>
#include <GL/glut.h>

std::mutex mutex;

rs2_vector accel;
rs2_vector gyro;
Eigen::Quaternionf q(1.0, 0.0, 0.0, 0.0);

Eigen::Vector3f gravity(0, -9.81, 0);

Eigen::Vector3f world_accel(0,0,0);
Eigen::Vector3f accel_raw(0,0,0);
Eigen::Vector3f accel_rot(0,0,0);
Eigen::Vector3f accel_invrot(0,0,0);

std::vector<Eigen::Vector3f> path{Eigen::Vector3f(0,0,0)};
Eigen::Vector3f pos(0,0,0);

void update_path(){
    world_accel
}

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

    glBegin(GL_LINES);
        //glColor3f(0.5, 0, 0); glVertex3f(0,0,0); glVertex3f(accel_raw[0], accel_raw[1], accel_raw[2]);
        //glColor3f(0, 0.5, 0); glVertex3f(0,0,0); glVertex3f(accel_rot[0], accel_rot[1], accel_rot[2]);
        glColor3f(0.5, 0.5, 0.5); glVertex3f(0,0,0); glVertex3f(world_accel[0], world_accel[1], world_accel[2]);
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

void display_thread(){
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowPosition(80, 80);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Orientation");
    glutDisplayFunc(display);
    init();
    glutTimerFunc(100, timer, 0);

    glutMainLoop();
}

int main(int argc, char * argv[]) try
{
    glutInit(&argc, argv);

    std::thread display = std::thread(display_thread);

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);
    rs2::pipeline_profile profile = pipe.start(cfg);
    rs2::device dev = profile.get_device();

    std::ofstream dump;
    dump.open ("readings.csv");

    std::ofstream path;
    path.open ("path.csv");

    while(true){
        auto frameset = pipe.wait_for_frames();

        mutex.lock();

        auto aframe =  frameset.first(RS2_STREAM_ACCEL).as<rs2::motion_frame>();
        auto gframe =  frameset.first(RS2_STREAM_GYRO).as<rs2::motion_frame>();

        accel = aframe.get_motion_data();
        gyro = gframe.get_motion_data();

	MadgwickAHRSupdateIMU(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);

        q = Eigen::Quaternionf(q0, q1, q2, q3);

        auto invrot = q.normalized().inverse().toRotationMatrix();
        auto rot = q.normalized().toRotationMatrix();

        accel_raw = Eigen::Vector3f (accel.x, accel.y, accel.z);
        accel_rot = rot * accel_raw;
        world_accel = accel_rot - gravity;

        std::cout << world_accel << std::endl;

        dump << world_accel[0] << "," << world_accel[1] << "," << world_accel[2]  << "," << 
            gyro.x << "," << gyro.y << "," << gyro.z  << ","  << 
            q0 << "," << q1 << "," << q2  << "," << q3  << std::endl;


        update_path(world_accel)

        mutex.unlock();
    }

    dump.close();
    display.join();
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


