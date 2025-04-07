import matplotlib.pyplot as plt  # type: ignore
import numpy as np  # type: ignore
from scipy.interpolate import CubicSpline
# from motorControl import move_car, coastAll

# had to comment out the plotting code because it would not compile on the car

fig, axd = plt.subplot_mosaic([['left', 'left', 'upper middle', 'upper right'],
                               ['left', 'left', 'lower middle', 'lower right']
                               ], figsize=(8, 4), layout="constrained")
fig.suptitle('Simulator with wheel speed display')
axd['left'].set_aspect('equal')
axd['left'].set_xlim(-5, 5)
axd['left'].set_ylim(-5, 5)
wheelName = ['fl', 'fr', 'rl', 'rr']
A = 0.240/2
B = 0.075/2
MAXSPEED = 100

plt.style.use('_mpl-gallery-nogrid')


# point is a tuple (x, y)
def plotPoint(point, setting):
    axd['left'].plot(point[0], point[1], setting)


# between two points
def plotArrow(p1, p2):
    delta = tuple(x - y for (x, y) in zip(p2, p1))
    axd['left'].quiver(p1[0], p1[1], delta[0], delta[1], width=0.003)


# returns list of points between 2 points
def linePoints(p1, p2, step_size, rate):
    rise = p2[1] - p1[1]
    run = p2[0] - p1[0]
    distance = np.sqrt(run**2 + rise**2)
    t_max = distance/(step_size*rate)
    t_line = np.linspace(0, 1, int(t_max*rate))

    return list(zip(run*t_line+p1[0], rise*t_line+p1[1]))


def fixAngles(angle1, angle2):
    while (abs(angle1 - angle2) > 180):
        if (angle1 < angle2):
            angle1 += 360
        else:
            angle2 += 360
    return angle1, angle2


# radius > 0 is CCW arc, radius < 0 is CW arc, returns list of points
def arcPoints(p1, p2, radius, step_size, rate):
    # calculating the midpoint and distance between p2 and p1
    diff = tuple(x - y for (x, y) in zip(p2, p1))
    midpoint = tuple((x + y)/2 for (x, y) in zip(p2, p1))
    distance = np.sqrt(diff[0]**2 + diff[1]**2)

    if (distance > 2*abs(radius)):
        raise Exception("No arc of radius "+str(abs(radius))
                        + " between the points given")

    # finding the center of the arc circle
    n_hat = (-diff[1]/distance, diff[0]/distance)
    if (radius < 0):
        n_hat = tuple(x * -1 for x in n_hat)
    normal = tuple(x * np.sqrt(radius**2 - (0.5*distance)**2) for x in n_hat)
    center = tuple(x + y for (x, y) in zip(midpoint, normal))

    # calculating the angles the arc will be around
    p1_angle = np.arctan2(p1[1]-center[1], p1[0]-center[0])*180/np.pi
    p2_angle = np.arctan2(p2[1]-center[1], p2[0]-center[0])*180/np.pi

    p1_angle, p2_angle = fixAngles(p1_angle, p2_angle)

    if (p1_angle == p2_angle):
        p2_angle += 360

    if (normal == (0.0, 0.0)):  # if drawing a semicircle
        if (radius < 0 and p1_angle < p2_angle or
           radius > 0 and p2_angle < p1_angle):
            temp = p2_angle
            p2_angle = p1_angle + 180
            p1_angle = temp + 180

    return circlePoints(center, abs(radius), step_size, rate,
                        p1_angle, p2_angle)


# returns list of points tracing a circle between 2 angles
def circlePoints(center, radius,  # center is a tuple (x, y)
                 step_size, rate,
                 start_angle, stop_angle):  # these are in degrees
    start = start_angle/360  # convert degree to number between 0 and 1
    stop = stop_angle/360
    t_max = 360/(step_size*rate)

    # functions for a circle (can take and return a list)
    def x(t): return radius * np.cos(2 * np.pi * t) + center[0]
    def y(t): return radius * np.sin(2 * np.pi * t) + center[1]

    # create a list of times and then a list of circle points
    t_circle = np.linspace(start, stop, abs(int(t_max*rate*(stop-start))))
    return list(zip(x(t_circle), y(t_circle)))


def splinePoints(xs, ys, step_size, rate):
    f = CubicSpline(xs, ys, bc_type='natural')
    a_billion = 100000  # a large number
    x = np.linspace(0, xs[-1], a_billion)
    y = f(x)
    dr = np.sqrt((x[1]-x[0])**2 + (y[1]-y[0])**2)
    step_ratio = int(step_size/dr)
    x_points = list(x[i] for i in range(0, len(x), step_ratio))
    y_points = list(y[i] for i in range(0, len(y), step_ratio))
    return list(zip(x_points, y_points))


# angle of arrow going into a path relative to the horizontal
def entryAngle(path):  # path is a list of points (x, y)
    if len(path) <= 1:
        raise Exception("step size too large to draw path")
    return 180/np.pi * \
        np.arctan2(path[1][1] - path[0][1], path[1][0] - path[0][0])


# angle of arrow coming out of a path relative to the horizontal
def exitAngle(path):
    return 180/np.pi * \
        np.arctan2(path[-1][1]-path[-2][1], path[-1][0]-path[-2][0])


# path is a tuple ex: ("line") or ("curve", 3) or ("semicircle", True)
def pathPoints(p1, p2, path, step_size, rate):
    if path[0] == "line":
        return linePoints(p1, p2, step_size, rate)
    elif path[0] == "curve":
        r = path[1]
        angle_step = step_size * 360/(2*np.pi) / abs(r)
        return arcPoints(p1, p2, r, angle_step, rate)
    elif path[0] == "semicircle":
        ccw = path[1]
        delta = tuple(x2 - x for (x2, x) in zip(p2, p1))
        dist = np.sqrt(delta[0]**2 + delta[1]**2)
        r = dist/2 if ccw else -dist/2
        angle_step = step_size * 360/(2*np.pi) / abs(r)
        return arcPoints(p1, p2, r, angle_step, rate)
    elif path[0] == "spline":
        xs = path[1]
        ys = path[2]
        return splinePoints(xs, ys, step_size, rate)
    raise Exception("type: "+str(path)+" is not a valid type")


# # draws arrows from a center to a list of points
# def doTankTurn(deltaT, spin_angle):
#     omega = spin_angle*np.pi/180/deltaT*1.2
#     print("angle, deltaT, omega: ", spin_angle, deltaT, omega)
#     speeds = wheelSpeeds(0, 90, omega, A, B)
#     print("speeds: ", speeds)
#     # move_car(speeds, deltaT)
#     for point in tt:
#         plotArrow(center, point)
#         # car_angle = np.arctan2(point[1]-center[1], point[0]-center[0])
#         # print("tt", car_angle)
#         speeds = wheelSpeeds(0, 90, omega, A, B)
#         speeds = list(speed*10 for speed in speeds)
#         plotWheels(axd, wheelName, speeds, MAXSPEED)
#         plt.pause(1/rate)


# # draws dots along a path, path is a tuple e.g. ("curve", 3)
# def plotPath(p1, p2, path, step_size, rate):
#     for point in pathPoints(p1, p2, path, step_size, rate):
#         plotPoint(point, "ro")
#         plt.pause(1/rate)


# # draws dots along a list of paths between points
# def plotShape(pts, paths, step_size, rate):
#     for i in range(len(paths)-1):
#         plotPath(pts[i], pts[i+1], paths[i], step_size, rate)
#     plotPath(pts[-1], pts[0], paths[-1], step_size, rate)


# draws arrows between a list of points
def tracePath(path, step_size, rate):
    spin_angle = (exitAngle(path) - entryAngle(path)) * np.pi/180
    # print(len(path), exitAngle(path), entryAngle(path))
    deltaT = len(path)/rate
    omega = spin_angle/deltaT*1.85     # some correction
    v_car = step_size * rate
    # print("deltaT, speeds: ", deltaT, speeds)
    # move_car(speeds, deltaT/2)
    for i in range(len(path)-1):
        plotArrow(path[i], path[i+1])
        # car_angle = np.arctan2(path[i+1][1] - path[i][1],
        #                        path[i+1][0] - path[i][0])
        # print("p: ", car_angle)
        plotWheels(axd, wheelName,
                   wheelSpeeds(v_car, np.pi/2, omega, A, B), MAXSPEED)
        plt.pause(1/rate)


def tracePathFaceOneWay(path, step_size, rate):
    for i in range(len(path)-1):
        delX = path[i+1][0]-path[i][0]
        delY = path[i+1][1]-path[i][1]
        theta = np.arctan2(delY, delX)
        # print("theta, delY, delX: ", theta, delY, delX)
        omega = 0
        v_car = step_size * rate/2
        speeds = wheelSpeeds(v_car, theta, omega, A, B)
        # move_car(speeds, 1/rate)


# draws arrows along a list of paths between points with tank turns
def traceShape(pts, paths, face_one_way, step_size, rate):
    shape = []
    turns = []

    print("paths: ", paths)
    # paths
    for i in range(len(pts)-1):
        shape.append(pathPoints(pts[i], pts[i+1], paths[i], step_size, rate))
    if len(paths) == len(pts):
        shape.append(pathPoints(pts[-1], pts[0], paths[-1], step_size, rate))



    # tank turn
    for i in range(len(shape)-1):
        start = exitAngle(shape[i])
        end = entryAngle(shape[i+1])
        start, end = fixAngles(start, end)
        spin_angle = (end - start)
        # center = shape[i][-1]
        deltaT = valmap(abs(spin_angle), 0, 360, 0, 2)
        # full 360 should be 2 seconds, trying to standarize spin speeds
        turns.append((deltaT, spin_angle
                      # ,circlePoints(center, 10000, 5, rate, start, end)
                      ))
        print(deltaT, spin_angle)
    turns.append((None, None))  # buffer for the zip

    for path, (deltaT, spin_angle) in zip(shape, turns):
        print("doing path at ", rate)
        if (face_one_way):
            tracePathFaceOneWay(path, step_size, rate)
        else:
            tracePath(path, step_size, rate)
        # coastAll(0.5)
        if (not face_one_way and spin_angle is not None):
            if (abs(spin_angle) > 10 and deltaT > 0):
                print("doing tank turn at ", rate, " steps per sec")
                # doTankTurn(deltaT, spin_angle)
                # coastAll(0.5)


# theta locked to 90 for now since the car faces in the direction of the path
# theta and omega are in radians
# speeds in m/s
def wheelSpeeds(vt, theta, omega, a, b):
    vty = vt * np.sin(theta)  # = vt
    vtx = vt * np.cos(theta)  # = 0
    speeds = [vty + vtx - omega * (a+b),
              vty - vtx + omega * (a+b),
              vty - vtx - omega * (a+b),
              vty + vtx + omega * (a+b)]
    return speeds


# equivalent of Arduino map()
def valmap(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


# plotWheels(graphs, wheelSpeed, maxSpeed, radius):
# use adk[0] for car's path plot
def plotWheels(axd, wheelName, wheelSpeed, maxSpeed, radius=1):
    for idx, k in enumerate(axd):
        if idx > 0:
            ang = valmap(wheelSpeed[idx-1], -maxSpeed, maxSpeed, np.pi, 0)
            dx = radius * np.cos(ang)
            dy = radius * np.sin(ang)
            axd[k].clear()
            axd[k].figure.set_figwidth(9)
            axd[k].set(xlim=(-1.5, 1.5),  xticks=[], yticks=[],
                       ylim=(-1.5, 1.5))
            axd[k].annotate(wheelName[idx - 1], (0.1, 0.5),
                            xycoords='axes fraction', va='center')
            axd[k].quiver(-0.1, 0, dy, dx, width=0.02,
                          pivot='mid', angles='uv',
                          scale_units='height', scale=2)
            axd[k].annotate(round(wheelSpeed[idx - 1], 2), (.7, 0.5),
                            xycoords='axes fraction', va='center')
