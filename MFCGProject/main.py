import copy
from math import sqrt

import numpy as np
import open3d as o3d

from bezier_spline import BezierSpline


def make_equals_triangles_method1(figure_from, figure_to):
    first_v = len(np.asarray(figure_to.vertices))
    first_t = len(np.asarray(figure_to.triangles))
    print(f"f_v:{first_v}, f_t:{first_t}")
    second_vertices = np.asarray(figure_from.vertices)
    second_v = len(second_vertices)
    second_triangles = np.asarray(figure_from.triangles)
    second_t = len(second_triangles)

    vertices = second_vertices.tolist()
    triangles = []

    while first_v != second_v:
        for i in range(0, len(second_triangles)):
            if first_v == second_v:
                break
            second_v += 1
            second_t += 2
            t = second_triangles[i]
            v1 = second_vertices[t[0]]
            v2 = second_vertices[t[1]]
            v3 = second_vertices[t[2]]
            center = [0, 0, 0]
            if v1[0] == v2[0] and v2[0] == v3[0]:
                center = [v1[0], (v1[1] + v2[1] + v3[1]) / 3, (v1[2] + v2[2] + v3[2]) / 3]
            elif v1[1] == v2[1] and v2[1] == v3[1]:
                center = [(v1[0] + v2[0] + v3[0]) / 3, v1[1], (v1[2] + v2[2] + v3[2]) / 3]
            elif v1[2] == v2[2] and v2[2] == v3[2]:
                center = [(v1[0] + v2[0] + v3[0]) / 3, (v1[1] + v2[1] + v3[1]) / 3, v1[2]]
            index = len(vertices)
            triangles.append([index, t[0], t[1]])
            triangles.append([index, t[0], t[2]])
            triangles.append([index, t[2], t[1]])
            vertices.append(center)
        second_triangles = triangles
        second_vertices = vertices

    figure_from.vertices = o3d.utility.Vector3dVector(np.asarray(vertices))
    figure_from.triangles = o3d.utility.Vector3iVector(np.asarray(triangles))

def make_equals_triangles_method2(figure_from, figure_to):
    first_v = len(np.asarray(figure_to.vertices))
    first_t = len(np.asarray(figure_to.triangles))
    print(f"f_v:{first_v}, f_t:{first_t}")
    second_vertices = np.asarray(figure_from.vertices)
    second_v = len(second_vertices)
    second_triangles = np.asarray(figure_from.triangles)
    second_t = len(second_triangles)

    vertices = second_vertices.tolist()
    triangles = second_triangles.tolist()

    while first_v != second_v:
        for i in range(0, len(second_triangles)):
            if first_v == second_v:
                break
            second_v += 1
            second_t += 2
            t = second_triangles[i]
            v1 = second_vertices[t[0]]
            v2 = second_vertices[t[1]]
            v3 = second_vertices[t[2]]
            index = len(vertices)
            triangles.append([index, t[0], t[1]])
            triangles.append([index, t[1], t[2]])
            vertices.append(v3)

    figure_from.vertices = o3d.utility.Vector3dVector(np.asarray(vertices))
    figure_from.triangles = o3d.utility.Vector3iVector(np.asarray(triangles))
    print(f"s_v:{len(vertices)}, s_t:{len(triangles)}")

def test():
    size = 0.1
    sphere = o3d.geometry.TriangleMesh.create_sphere(size)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color([1, 0, 0])



    box = o3d.geometry.TriangleMesh.create_box(2*size, 2*size, 2*size)
    box_v = np.asarray(box.vertices)
    for i in range(0, len(box_v)):
        box_v[i] = box_v[i] - size*np.asarray([1, 1, 1])
    box.vertices = o3d.utility.Vector3dVector(np.asarray(box_v))
    make_equals_triangles_method2(box, sphere)
    # box.vertices = o3d.utility.Vector3dVector(np.asarray([[2,1,0],[2,6,0],[7,1,0],[7, 6, 0]]))
    # box.triangles = o3d.utility.Vector3iVector(np.asarray([[0, 1, 2], [1, 2, 3]]))
    box.compute_vertex_normals()
    box.paint_uniform_color([0, 1, 0])
    t = Test(box, sphere)
    cone = o3d.geometry.TriangleMesh.create_cone(size, 2*size)
    cone_v = np.asarray(cone.vertices)
    for i in range(0, len(cone_v)):
        cone_v[i] = cone_v[i] - size * np.asarray([0, 0, 1])
    cone.vertices = o3d.utility.Vector3dVector(np.asarray(cone_v))
    print(f"{len(cone_v)} {len(np.asarray(cone.triangles))}")
    make_equals_triangles_method2(cone, sphere)
    cone.compute_vertex_normals()
    cone.paint_uniform_color([0, 1, 0])
    t = Test(cone, sphere)
    # o3d.visualization.draw_geometries([sphere, cone], "Test", 800, 800, mesh_show_wireframe=True,mesh_show_back_face=True)

def point_abs(a):
    res = 0
    for i in range(0, len(a)):
        res += a[i] ** 2
    return sqrt(res)

class Test:
    def make_point_pairs(self):
        new_from_v = []
        for i in range(0, len(self.to_v)):
            point = self.to_v[i]
            delta_min = point_abs(self.from_v[0] - point)
            index_min = 0
            for j in range(1, len(self.from_v)):
                point2 = self.from_v[j]
                delta = point_abs(point2 - point)
                if delta < delta_min:
                    delta_min = delta
                    index_min = j
            new_from_v.append(self.from_v[index_min])
            self.from_v.remove(self.from_v[index_min])
        self.from_v = np.array(new_from_v)



    def __init__(self, box, sphere):
        self.t = 0
        self.color_from = np.asarray([0, 1, 0])
        self.color_to = np.asarray([1, 0, 0])
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.box = box
        self.from_v = np.asarray(box.vertices).tolist()
        input_points = o3d.geometry.PointCloud()
        input_points.points = o3d.utility.Vector3dVector(self.from_v)
        input_points.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(0, len(self.from_v))])
        self.current = copy.deepcopy(box)
        self.sphere = sphere
        self.to_v = np.asarray(sphere.vertices)
        self.make_point_pairs()
        self.current.vertices = o3d.utility.Vector3dVector(self.from_v)
        self.current.compute_vertex_normals()
        self.current.triangles = self.sphere.triangles
        self.current.compute_vertex_normals()
        self.current.compute_triangle_normals()

        self.vis.create_window("3D Morphing", 800, 800, 100, 100)
        self.vis.register_key_callback(65, self.your_update_function)
        self.vis.add_geometry(self.current)
        self.vis.run()
        self.vis.destroy_window()

    def your_update_function(self, vis):
        self.t += 0.05
        if self.t >= 1:
            self.t = 1
        print(self.t)
        ver = np.asarray(self.current.vertices)
        """triangle_index = int((self.t/2 + 0.5) * len(np.asarray(self.sphere.triangles)))
        triangle = np.asarray(self.current.triangles)
        triangle_to = np.asarray(self.sphere.triangles)
        for i in range(0, triangle_index):
            triangle[i] = triangle_to[i]
        self.current.triangles = o3d.utility.Vector3iVector(triangle)"""
        color = (1 - self.t)*self.color_from + self.t * self.color_to
        for i in range(0, len(ver)):
            bezier = BezierSpline([self.from_v[i], self.to_v[i]])
            ver[i] = bezier.run(self.t, 0)
        self.current.vertices = o3d.utility.Vector3dVector(ver)
        self.current.paint_uniform_color(color)
        self.current.compute_vertex_normals()
        self.current.compute_triangle_normals()
        vis.update_geometry(self.current)
        vis.update_renderer()
        vis.poll_events()
        vis.run()
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    test()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
