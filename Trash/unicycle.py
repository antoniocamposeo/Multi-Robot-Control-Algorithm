import numpy as np

'''
simple gain controller k1 
'''
zeta = 0.5
a = 5


def k1_circ(v_d, w_d):
    global zeta, a
    return 2 * zeta * a


def k1(v_d, w_d):
    global zeta, a
    return 2 * zeta * np.sqrt(v_d ** 2 + a * w_d ** 2)


'''
simple costant gain controller k3
'''


def k3_circ(v_d, w_d):
    global zeta, a
    return 2 * zeta * a


def k3(v_d, w_d):
    global zeta, a
    return 2 * zeta * np.sqrt(v_d ** 2 + a * w_d ** 2)


'''
Control Law implementation. k1 and k3 functions 
are used to possibly implement time varyng gains, whereas the gain k2 is set in the function
'''


def control(e, v_d, w_d):
    # k2 = 0.5;
    global a, zeta
    k2 = a

    u_1 = - k1(v_d, w_d) * e[0]

    if e[2] == 0:
        u_2 = -k2 * v_d * e[1] - k3(v_d, w_d) * e[2]
    else:
        u_2 = -k2 * v_d * np.sin(e[2]) / e[2] * e[1] - k3(v_d, w_d) * e[2]

    return np.array([u_1, u_2])


'''
Error dynamics. This function is used odeint to 
simulate the close loop system
'''


# v_d is computed in [0, 10]s
# w_d is computed in [0, 10]s
# time is vector in [0, 10]s
def unicycle_error_model(e, t, v_d, w_d, time):
    # T is the index in the 'time' vector that is
    # closest to the current integration time 't'
    T = np.searchsorted(time, t)
    if (T >= len(time)):
        T = len(time) - 1
    # e[1] is e_2
    u_1, u_2 = control(e, v_d[T], w_d[T])
    edot_1 = u_1 + e[1] * (w_d[T] - u_2)
    edot_2 = v_d[T] * np.sin(e[2]) - e[0] * (w_d[T] - u_2)
    edot_3 = u_2
    return [edot_1, edot_2, edot_3]


def R(theta):
    return np.array([[np.cos(theta), np.sin(theta), 0], [-np.sin(theta), np.cos(theta), 1], [0., 0., 1.]])


def from_polar_to_cartesian(p):
    rho = p[0]
    gamma = p[1]
    delta = p[2]
    x = rho * np.cos(delta + np.pi)
    y = rho * np.sin(delta + np.pi)
    theta = delta - gamma
    return x, y, theta


def from_cartesian_to_polar(q):
    x = q[0]
    y = q[1]
    theta = q[2]
    rho = np.sqrt(x ** 2 + y ** 2)
    gamma = np.arctan2(y, x) - theta + np.pi
    delta = gamma + theta
    return rho, gamma, delta
