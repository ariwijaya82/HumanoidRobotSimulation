from Bezier import Bezier
import numpy as np
from numpy import savetxt
from numpy import array as a
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

## Set Up Figure ##
fig = plt.figure(figsize=plt.figaspect(0.5))

### Bezier Curve Raising Foot ###
points_set_1 = a([[0, 7.5, 6], [0, 7.5, 7], [-8, 7.5, 7], [-12, 7.5, 7.5], [-20, 7.5, 13.5]])
t_points = np.arange(0, 1, 0.05)
curve_set_1 = Bezier.Curve(t_points, points_set_1)

## First Plot Figure ##
ax = fig.add_subplot(1, 2, 1, projection='3d')
ax.plot(curve_set_1[:, 0], curve_set_1[:, 1], curve_set_1[:, 2])
ax.plot(curve_set_1[:, 0], curve_set_1[:, 1], curve_set_1[:, 2], 'o', markersize = 4.5)
print(curve_set_1)
np.savetxt('RaisingFoot.csv', curve_set_1, fmt = '%s', delimiter=",")  
ax.set_title('Angkat Kaki Robot')
ax.plot(points_set_1[:, 0], points_set_1[:, 1], points_set_1[:, 2], 'o:')


### Bezier Curve Kick Forward ###
points_set_1 = a([[-20, 7.5, 13.5], [-10, 7.5, 8], [40, 7.5, 8], [60, 7.5, 10.5]])
t_points = np.arange(0, 1, 0.05)
curve_set_1 = Bezier.Curve(t_points, points_set_1)

## Second Plot Figure ##
ax = fig.add_subplot(1, 2, 2, projection='3d')
ax.plot(curve_set_1[:, 0], curve_set_1[:, 1], curve_set_1[:, 2])
ax.plot(curve_set_1[:, 0], curve_set_1[:, 1], curve_set_1[:, 2], 'o', markersize = 4.5)
print(curve_set_1)
np.savetxt('KickForward.csv', curve_set_1, fmt = '%s', delimiter=",")   
ax.set_title('Menendang Bola')
ax.plot(points_set_1[:, 0], points_set_1[:, 1], points_set_1[:, 2], 'o:')

plt.savefig('Bezier_Sampling.png')

plt.show()

help(Bezier)