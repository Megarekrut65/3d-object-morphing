import copy

import numpy as np
import open3d as o3d


def test():
    size = 0.1
    sphere = o3d.geometry.TriangleMesh.create_sphere(size)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color([1, 0, 0])

    first_v = len(np.asarray(sphere.vertices))
    first_t = len(np.asarray(sphere.triangles))
    print(f"f_v:{first_v}, f_t:{first_t}")

    box = o3d.geometry.TriangleMesh.create_box(2*size, 2*size, 2*size)
    box_v = np.asarray(box.vertices)
    for i in range(0, len(box_v)):
        box_v[i] = box_v[i] - size*np.asarray([1, 1, 1])
    box.vertices = o3d.utility.Vector3dVector(np.asarray(box_v))

    # box.vertices = o3d.utility.Vector3dVector(np.asarray([[2,1,0],[2,6,0],[7,1,0],[7, 6, 0]]))
    # box.triangles = o3d.utility.Vector3iVector(np.asarray([[0, 1, 2], [1, 2, 3]]))
    second_vertices = np.asarray(box.vertices)
    second_v = len(second_vertices)
    second_triangles = np.asarray(box.triangles)
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
            center = [0,0,0]
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

    box.vertices = o3d.utility.Vector3dVector(np.asarray(vertices))
    box.triangles = o3d.utility.Vector3iVector(np.asarray(triangles))
    box.compute_vertex_normals()
    box.paint_uniform_color([0, 1, 0])
    print(second_v, second_t)

    o3d.visualization.draw_geometries([sphere, box], "Test", 800, 800, mesh_show_wireframe=True,
                                      mesh_show_back_face=True)

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    test()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
