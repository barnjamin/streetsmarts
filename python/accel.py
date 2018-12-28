#! /usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import quaternion

np.random.seed(42)

#ax, ay, az, gx, gy, gz, q0, q1, q2, q3


gravity = np.array([0,-9.81,0])

def accel_from_file(path):
    accel = []
    f = open(path, 'r')
    for line in f:
        chunks = line.split(',') 
        accel.append([float(chunk) for chunk in chunks])

    return np.asarray(accel).T

def accel_to_file(path, accel):
    f = open(path, 'f')
    for line in accel:
        f.write(line.join(',')+'\n')

    f.close()

def rotation_accel(length):
    accel = np.empty((3,length))

    # Rotate in one direction by certain rate
    for x in range(length):
        rotation = np.quaternion(1.0, x%180, 0.0, 0.0)

        # turn rotation quaternion into transformation
        mat = quaternion.as_rotation_matrix(rotation)

        # rotate gravity vector by quaternion
        accel[:, x]= np.dot(mat, gravity)

    return accel

def random_accel(length):
    accel = np.empty((3,length))

    for x in range(length):
        accel[:, x]  = (np.random.rand(3) - 0.5) * np.random.rand(3) 

    return accel

def path_from_accel(accel, length, timestep):
    pos     = np.zeros(3) 
    vel     = [0.0,0.0,0.0]
    path    = np.empty((3, length))
    path[:, 0] = [0,0,0] 

    for idx in range(length):
        a = accel[:,idx]

        vel[0] = vel[0] + a[0]*timestep
        vel[1] = vel[1] + a[1]*timestep
        vel[2] = vel[2] + a[2]*timestep

        pos[0] = pos[0] + vel[0] * timestep + (a[0] * timestep**2)/2
        pos[1] = pos[1] + vel[1] * timestep + (a[1] * timestep**2)/2
        pos[2] = pos[2] + vel[2] * timestep + (a[2] * timestep**2)/2
            
        path[:, idx] = pos

    return path


def path_from_file(filepath):
    path = []
    f = open(filepath, 'r')
    for line in f:
        chunks = line.split(',') 
        path.append([float(chunk) for chunk in chunks])

    return np.array(path).T

def update_lines(num, dataLines, lines):
    for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
        line.set_data(data[0:2, :num])
        line.set_3d_properties(data[2, :num])
    return lines




if __name__ == "__main__":

    # Attaching 3D axis to the figure
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    # Setting the axes properties
    ax.set_xlim3d([-10.0, 10.0])
    ax.set_xlabel('X')

    ax.set_ylim3d([-10.0, 10.0])
    ax.set_ylabel('Y')

    ax.set_zlim3d([-10.0, 10.0])
    ax.set_zlabel('Z')

    ax.set_title('3D Test')


    frequency = 20
    timestep =  1/frequency

    #accel = rotation_accel(length) 
    #accel = accel_from_file("/home/ben/streetsmarts/build/readings.csv")
    #path = path_from_accel(accel, length, timestep)
    path = path_from_file("/home/ben/streetsmarts/build/path.csv")

    length = path.size

    lines = [ax.plot(path[0, 0:1], path[1, 0:1], path[2, 0:1])[0]]

    line_ani = animation.FuncAnimation(fig, update_lines, length, fargs=([path], lines), blit=False)

    plt.show()
