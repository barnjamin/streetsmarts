#pragma once
#include <mutex>
#include <thread>
#include <Eigen/Geometry>
#include <GL/glut.h>
#include "pose.h"


class Display {
    Pose * pose;
    std::thread dthread;

public:

    // Sets up global attributes like clear color and drawing color, and sets up
    // the desired projection and modelview matrices.
    Display(int argc, char * argv[], Pose *p);
    virtual ~Display();

    void start();
};
