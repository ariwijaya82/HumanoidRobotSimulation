import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from bezier import Bezier
import fk
import ik

def save_data(name, data):
    np.savetxt(name, data, fmt = '%s', delimiter=",")

if __name__ == '__main__':
    # raising foot
    points_set_1 = np.array([[0, 7.5, 6], [0, 7.5, 7], [-8, 7.5, 7], [-12, 7.5, 7.5], [-20, 7.5, 13.5]])
    t_points_1 = np.arange(0, 1, 0.01)
    curve_set_1 = Bezier.Curve(t_points_1, points_set_1)
    # save_data('test.csv', curve_set_1)

    # kicking
    points_set_2 = np.array([[-20, 7.5, 13.5], [-10, 7.5, 8], [40, 7.5, 8], [60, 7.5, 10.5]])
    t_points_2 = np.arange(0, 1, 0.01)
    curve_set_2 = Bezier.Curve(t_points_2, points_set_2)

    # plotting
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.plot(points_set_1[:, 0], points_set_1[:, 1], points_set_1[:, 2], 'o:b')
    ax.plot(curve_set_1[:, 0], curve_set_1[:, 1], curve_set_1[:, 2], '-b')

    ax.plot(points_set_2[:, 0], points_set_2[:, 1], points_set_2[:, 2], 'o:g')
    ax.plot(curve_set_2[:, 0], curve_set_2[:, 1], curve_set_2[:, 2], '-g')

    inverse_kinematic = []
    for data in curve_set_1:
        inverse_kinematic.append(ik.compute_ik_degree(data[0], data[1], data[2]))
    for data in curve_set_2:
        inverse_kinematic.append(ik.compute_ik_degree(data[0], data[1], data[2]))


    forward_kinematic = []
    for data in inverse_kinematic:
        forward_kinematic.append(fk.compute_fk(data))

    result = np.array(forward_kinematic)
    # save_data('result.csv', result)
    ax.plot(result[:,0], result[:,1], result[:,2], '-r')
    ax.set_ylim3d([0,14])

    plt.title("Path Planning")
    plt.show()