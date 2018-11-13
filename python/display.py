#! /usr/bin/env python

import copy
import numpy as np
from open3d import *

if __name__ == "__main__":

    print("Testing mesh in open3d ...")
    #mesh = read_triangle_mesh("latest/scene/integrated.ply")
    mesh = read_triangle_mesh("../build/fragment-0.ply")
    draw_geometries([mesh])
    mesh = read_triangle_mesh("../build/fragment-1.ply")
    draw_geometries([mesh])
    mesh = read_triangle_mesh("../build/fragment-2.ply")
    draw_geometries([mesh])

    import sys
    sys.exit()

    print("Try to render a mesh with normals (exist: " +
            str(mesh.has_vertex_normals()) +
            ") and colors (exist: " + str(mesh.has_vertex_colors()) + ")")
    draw_geometries([mesh])
    print("A mesh with no normals and no colors does not seem good.")

    print("Computing normal and rendering it.")
    mesh.compute_vertex_normals()
    print(np.asarray(mesh.triangle_normals))
    draw_geometries([mesh])

    print("We make a partial mesh of only the first half triangles.")
    mesh1 = copy.deepcopy(mesh)
    mesh1.triangles = Vector3iVector(
            np.asarray(mesh1.triangles)[:len(mesh1.triangles)//2, :])
    mesh1.triangle_normals = Vector3dVector(
            np.asarray(mesh1.triangle_normals)
            [:len(mesh1.triangle_normals)//2, :])
    print(mesh1.triangles)
    draw_geometries([mesh1])

    print("Painting the mesh")
    mesh1.paint_uniform_color([1, 0.706, 0])
    draw_geometries([mesh1])
