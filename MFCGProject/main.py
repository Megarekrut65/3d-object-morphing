
import numpy as np
import open3d as o3d

from morphing import Morph3D


def get_centered_box(size):
    box = o3d.geometry.TriangleMesh.create_box(2 * size, 2 * size, 2 * size)
    box_v = np.asarray(box.vertices)
    for i in range(0, len(box_v)):
        box_v[i] = box_v[i] - size * np.asarray([1, 1, 1])
    box.vertices = o3d.utility.Vector3dVector(np.asarray(box_v))
    box.compute_vertex_normals()

    return box


def get_centered_cone(size):
    cone = o3d.geometry.TriangleMesh.create_cone(size, 2 * size)
    cone_v = np.asarray(cone.vertices)
    for i in range(0, len(cone_v)):
        cone_v[i] = cone_v[i] - size * np.asarray([0, 0, 1])
    cone.vertices = o3d.utility.Vector3dVector(np.asarray(cone_v))
    cone.compute_vertex_normals()

    return cone


def get_centered_sphere(size):
    sphere = o3d.geometry.TriangleMesh.create_sphere(size)
    sphere.compute_vertex_normals()

    return sphere


def get_centered_cylinder(size):
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(size, 2 * size)
    cylinder.compute_vertex_normals()

    return cylinder


def get_centered_icosahedron(size):
    icosahedron = o3d.geometry.TriangleMesh.create_icosahedron(size)
    icosahedron.compute_vertex_normals()

    return icosahedron


def main():
    size = 0.1

    # size can be difference
    sphere = get_centered_sphere(size)
    icosahedron = o3d.geometry.TriangleMesh.create_icosahedron(size/2)
    cylinder = get_centered_cylinder(size)
    box = get_centered_box(size)
    cone = get_centered_cone(size)

    t = Morph3D(box, cylinder)
    t.run()
    t = Morph3D(cylinder, cone)
    t.run()
    t = Morph3D(cone, sphere)
    t.run()
    t = Morph3D(sphere, icosahedron)
    t.run()


if __name__ == '__main__':
    main()
