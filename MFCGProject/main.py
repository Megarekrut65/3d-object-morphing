import copy
from math import sqrt

import numpy as np
import open3d as o3d

from bezier_spline import BezierSpline

def point_abs(a):
    res = 0
    for i in range(0, len(a)):
        res += a[i] ** 2
    return sqrt(res)

def find_closest_point_index(from_point, to_point):
    min_index = 0
    min_distance = point_abs(from_point - to_point[0])
    for j in range(1, len(to_point)):
        distance = point_abs(from_point - to_point[j])
        if distance < min_distance:
            min_distance = distance
            min_index = j
    return min_index

def make_mesh_isomorphic_to_other(from_obj, to_obj):
    to_ver = np.asarray(to_obj.vertices)
    from_ver = np.asarray(from_obj.vertices).tolist()

    from_current_ver = np.asarray(from_obj.vertices).tolist()
    new_ver = []

    for i in range(0, len(to_ver)):
        point = to_ver[i]
        if len(from_current_ver) == 0:
            new_ver.append(from_ver[find_closest_point_index(point, from_ver)])
            continue
        item = from_current_ver[find_closest_point_index(point, from_current_ver)]
        from_current_ver.remove(item)
        new_ver.append(item)

    new_obj = copy.deepcopy(from_obj)
    new_obj.vertices = o3d.utility.Vector3dVector(np.asarray(new_ver))
    new_obj.triangles = to_obj.triangles
    new_obj.compute_vertex_normals()
    new_obj.compute_triangle_normals()

    return new_obj


def main():
    size = 0.1
    sphere = o3d.geometry.TriangleMesh.create_sphere(size)
    sphere.compute_vertex_normals()
    cyl = o3d.geometry.TriangleMesh.create_cylinder(size, 2 * size)
    cyl.compute_vertex_normals()
    t = Morpher3D(cyl, sphere)

    box = o3d.geometry.TriangleMesh.create_box(2 * size, 2 * size, 2 * size)
    box_v = np.asarray(box.vertices)
    for i in range(0, len(box_v)):
        box_v[i] = box_v[i] - size * np.asarray([1, 1, 1])
    box.vertices = o3d.utility.Vector3dVector(np.asarray(box_v))
    t = Morpher3D(box, sphere)

    cone = o3d.geometry.TriangleMesh.create_cone(size, 2 * size)
    cone_v = np.asarray(cone.vertices)
    for i in range(0, len(cone_v)):
        cone_v[i] = cone_v[i] - size * np.asarray([0, 0, 1])
    cone.vertices = o3d.utility.Vector3dVector(np.asarray(cone_v))
    t = Morpher3D(cone, sphere)
    # o3d.visualization.draw_geometries([sphere, cone], "Test", 800, 800, mesh_show_wireframe=True,mesh_show_back_face=True)



class Morpher3D:

    def __init__(self, from_obj, to_obj):
        self.t = 0
        self.delta = 0.05

        self.to_color = np.asarray([1, 0, 0])
        self.to_obj = to_obj
        self.to_ver = np.asarray(self.to_obj.vertices)

        self.from_color = np.asarray([0, 1, 0])
        self.from_obj = from_obj

        self.iso = make_mesh_isomorphic_to_other(self.from_obj, self.to_obj)
        self.from_ver = np.asarray(self.iso.vertices)

        self.current_obj = copy.deepcopy(self.from_obj)
        self.current_obj.paint_uniform_color(self.from_color)
        self.current_obj.compute_vertex_normals()
        self.current_obj.compute_triangle_normals()

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("3D Morphing", 800, 800, 100, 100)
        self.vis.register_key_callback(65, self.update_function)
        self.vis.add_geometry(self.current_obj)
        self.vis.run()
        self.vis.destroy_window()

    def update_function(self, vis):
        if self.t < self.delta:
            self.current_obj.vertices = self.iso.vertices
            self.current_obj.triangles = self.iso.triangles
        self.t += self.delta
        if self.t >= 1:
            self.t = 1
        ver = np.asarray(self.current_obj.vertices)
        for i in range(0, len(ver)):
            ver[i] = (1 - self.t) * self.from_ver[i] + self.t * self.to_ver[i]
        self.current_obj.vertices = o3d.utility.Vector3dVector(ver)

        color = (1 - self.t) * self.from_color + self.t * self.to_color
        self.current_obj.paint_uniform_color(color)
        self.current_obj.compute_vertex_normals()
        self.current_obj.compute_triangle_normals()

        vis.update_geometry(self.current_obj)
        vis.update_renderer()
        vis.poll_events()
        vis.run()


if __name__ == '__main__':
    main()
