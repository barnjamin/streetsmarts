#include <iostream>
#include <mutex>
#include <thread>
#include <Eigen/Geometry>
#include <GL/glut.h>
#include "display.h"
#include <functional>

Pose * gpose;

// Clears the window and draws the torus.
void display() {

    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
    gluLookAt(4, 6, 5, 0, 0, 0, 0, 1, 0);

    auto q = gpose->GetOrientation();
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



Display::Display(int argc, char * argv[], Pose* p) : pose(p)
{
    gpose = pose;
    glutInit(&argc, argv);
}

void Display::start(){
    dthread = std::thread(display_thread);
}

Display::~Display()
{ 
}
