import copy
from math import sqrt

import numpy as np
import open3d as o3d


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
    from_ver = np.asarray(from_obj.vertices)

    new_ver = np.zeros((len(to_ver), 3))

    indexes = []
    for i in range(0, len(from_ver)):
        point = from_ver[i]
        index = find_closest_point_index(point, to_ver)
        new_ver[index] = from_ver[i]
        indexes.append(index)

    for i in range(0, len(to_ver)):
        if i in indexes:
            continue
        point = to_ver[i]
        new_ver[i] = from_ver[find_closest_point_index(point, from_ver)]

    new_obj = copy.deepcopy(from_obj)
    new_obj.vertices = o3d.utility.Vector3dVector(np.asarray(new_ver))
    new_obj.triangles = to_obj.triangles
    new_obj.compute_vertex_normals()
    new_obj.compute_triangle_normals()

    return new_obj


class Morph3D:

    def __init__(self, from_obj, to_obj, delta=0.03):
        self.t = 0
        self.delta = delta

        self.to_color = np.asarray([1, 0, 0])
        self.from_color = np.asarray([0, 1, 0])
        self.from_obj = from_obj
        self.to_obj = to_obj

        # adds additional vertices and triangles to the smaller shape
        from_obj_len = len(np.asarray(from_obj.vertices))
        to_obj_len = len(np.asarray(to_obj.vertices))

        if from_obj_len < to_obj_len:
            self.from_obj = make_mesh_isomorphic_to_other(from_obj, to_obj)
        elif from_obj_len > to_obj_len:
            self.to_obj = make_mesh_isomorphic_to_other(to_obj, from_obj)

        self.from_ver = np.asarray(self.from_obj.vertices)
        self.to_ver = np.asarray(self.to_obj.vertices)

        self.current_obj = copy.deepcopy(from_obj)
        self.current_obj.paint_uniform_color(self.from_color)
        self.current_obj.compute_vertex_normals()
        self.current_obj.compute_triangle_normals()

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.is_started = False

    def run(self, as_animation=True):
        self.vis.create_window("3D Morphing", 800, 800, 100, 100)

        if as_animation:
            self.vis.register_key_callback(32, self.__play_pause_anim)
            self.vis.register_animation_callback(self.__update_function)
        else:
            self.is_started = True
            self.vis.register_key_callback(32, self.__update_function)

        self.vis.add_geometry(self.current_obj)
        self.vis.run()
        self.vis.destroy_window()

    def __play_pause_anim(self, _):
        self.is_started = not self.is_started

    def __update_function(self, vis):
        if not self.is_started:
            return

        if self.t < self.delta:
            self.current_obj.vertices = self.from_obj.vertices
            self.current_obj.triangles = self.to_obj.triangles

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
