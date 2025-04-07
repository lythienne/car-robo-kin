from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt

#plt.style.use('seaborn-poster')


x = [0, 0.8, 2]
y = [0, 1.5, 0]

# use bc_type = 'natural' adds the constraints as we described above
f = CubicSpline(x, y, bc_type='natural')
x_new = np.linspace(0, 3, 100000)
y_new = f(x_new)


plt.figure(figsize = (10,8))
plt.plot(x_new, y_new, 'b')
plt.plot(x, y, 'ro')
plt.title('Cubic Spline Interpolation')
plt.xlabel('x')
plt.ylabel('y')
plt.show()