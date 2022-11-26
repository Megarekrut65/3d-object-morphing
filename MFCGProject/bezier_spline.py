import numpy as np


class BezierSpline:
    def __init__(self, points):
        self.np_points = np.array(points)
        n = len(self.np_points) - 1  # spline count

        self.control_points = self.__find_control_points(self.np_points, n)

    def run(self, t, spline_number):
        return self.__bezier_item(t, self.np_points[spline_number],  # pi
                                  self.control_points[2 * spline_number],  # ai
                                  self.control_points[2 * spline_number + 1],  # bi
                                  self.np_points[spline_number + 1])

    def __find_control_points(self, np_points, n):
        (_, dimension) = np_points.shape
        # create coefficient matrix and vector b
        matrix = np.zeros((2 * n, 2 * n), np.float)
        vector = np.zeros((2 * n, dimension), np.float)
        for i in np.arange(1, 2 * n - 2, 2):
            matrix[i, i - 1] = 1
            matrix[i, i] = -2
            matrix[i, i + 1] = 2
            matrix[i, i + 2] = -1
            matrix[i + 1, i] = 1
            matrix[i + 1, i + 1] = 1

            vector[i] = np.zeros(dimension, np.float)
            vector[i + 1] = 2 * np_points[i // 2 + 1]

        matrix[0, 0] = 2
        matrix[0, 1] = -1
        matrix[2 * n - 1, 2 * n - 2] = -1
        matrix[2 * n - 1, 2 * n - 1] = 2

        vector[0] = np_points[0]
        vector[2 * n - 1] = np_points[n]

        return np.linalg.solve(matrix, vector)

    def __bezier_item(self, t, pi, ai, bi, pi1):
        return (1 - t) ** 3 * pi + 3 * (1 - t) ** 2 * t * ai + 3 * (1 - t) * t ** 2 * bi + t ** 3 * pi1
