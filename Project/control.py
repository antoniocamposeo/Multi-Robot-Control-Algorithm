import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import Project.utils as utils


class Control:
    def __init__(self):
        self.v = None
        self.w = None
        self.w_arr = []
        self.v_arr = []
        self.t = None

    def control(self, *args):
        v = self.control_v(args)
        w = self.control_w(args)
        self.v_arr.append(v)
        self.w_arr.append(w)
        return v, w

    def control_v(self, *args):
        pass

    def control_w(self, *args):
        pass

    def model(self, q, t, *args):
        pass

    def get_v(self):
        len_v = len(self.v_arr)
        len_t = len(t)
        if len_v < len_t:
            diff = len_t - len_v  # len new vect to add
            vec_v = self.v_arr[-1] * np.ones(diff)
            return np.concatenate((np.array(self.v_arr), vec_v), axis=None)
        elif len_v == len_t:
            return self.v_arr

    def get_w(self):
        len_w = len(self.w_arr)
        len_t = len(t)
        if len_w < len_t:
            diff = len_t - len_w  # len new vect to add
            vec_w = self.w_arr[-1] * np.ones(diff)
            return np.concatenate((np.array(self.w_arr), vec_w), axis=None)
        elif len_w == len_t:
            return self.w_arr

    def get_time(self):
        return self.t

    def set_time(self, t):
        self.t = t


class TrajectoryTracking_Control(Control):

    def __init__(self, *args):
        self.name = 'TrajectoryTracking'
        self.a = args[0]
        self.zeta = args[1]
        self.k_2 = args[2]
        super(TrajectoryTracking_Control, self).__init__()

    def k(self, v_d, w_d):
        return 2 * self.zeta * np.sqrt(v_d ** 2 + self.a * w_d ** 2)

    def control(self, e, v_d, w_d):
        self.k_2 = self.a
        u_1 = - self.k(v_d, w_d) * e[0]
        if e[2] == 0:
            u_2 = -self.k_2 * v_d * e[1] - self.k(v_d, w_d) * e[2]
        else:
            u_2 = -self.k_2 * v_d * np.sin(e[2]) / e[2] * e[1] - self.k(v_d, w_d) * e[2]
        return np.array([u_1, u_2])

    '''
    Unicycle error model
    '''

    def model(self, q, t, *args):
        # T is the index in the 'time' vector that is
        # closest to the current integration time 't'
        x = q[0]
        y = q[1]
        theta = q[2]
        x_d, y_d, theta_d, v_d, w_d, time = args
        T = np.searchsorted(time, t)
        if (T >= len(time)):
            T = len(time) - 1
        e = [x - x_d[T], y - y_d[T], theta - theta_d[T]]
        u_1, u_2 = self.control(e, v_d[T], w_d[T])
        edot_1 = u_1 + e[1] * (w_d[T] - u_2)
        edot_2 = v_d[T] * np.sin(e[2]) - e[0] * (w_d[T] - u_2)
        edot_3 = u_2
        return [edot_1, edot_2, edot_3]


class I_O_Linearization_Control(Control):

    def __init__(self, *args):
        self.name = 'IOLinearization'
        self.k_1 = args[0]
        self.k_2 = args[1]
        self.k_1f = args[2]
        self.k_2f = args[3]
        self.b = None
        super(I_O_Linearization_Control, self).__init__()

    def control_v(self, *args):
        y1, y2, y1d, y2d, doty1d, doty2d = args[0]
        u_1 = self.k_1f * doty1d + self.k_1 * (y1d - y1)
        return u_1

    def control_w(self, *args):
        y1, y2, y1d, y2d, doty1d, doty2d = args[0]
        u_2 = self.k_2f * doty2d + self.k_2 * (y2d - y2)
        return u_2

    def set_b(self, b):
        self.b = b

    def get_b(self):
        return self.b

    '''
       Unicycle linearized model
    '''

    def model(self, q, t, *args):
        # T is the index in the 'time' vector that is
        # closest to the current integration time 't'
        y1d, y2d, doty1d, doty2d, time = args
        T = np.searchsorted(time, t)
        if (T >= len(time)):
            T = len(time) - 1
        y1 = q[0]
        y2 = q[1]
        theta = q[2]
        (u_1, u_2) = self.control(y1, y2, y1d[T], y2d[T], doty1d[T], doty2d[T])
        doty_1 = u_1
        doty_2 = u_2
        dottheta = u_2 / self.b * np.cos(theta) - u_1 / self.b * np.sin(theta)
        return [doty_1, doty_2, dottheta]


class CartesianRegulator_Control(Control):

    def __init__(self, *args):
        self.name = 'CartesianRegulator'
        self.k_1 = args[0]
        self.k_2 = args[1]
        super(CartesianRegulator_Control, self).__init__()

    def control_v(self, *args):
        e_x, e_y, e_theta = args[0]
        v = -self.k_1 * (e_x * np.cos(e_theta) + e_y * np.sin(e_theta))
        return v

    def control_w(self, *args):
        e_x, e_y, e_theta = args[0]
        w = self.k_2 * (np.arctan2(e_y, e_x) + np.pi - e_theta)
        return w

    def model(self, q, t, *args):
        x_d, y_d, theta_d, time = args
        T = np.searchsorted(time, t)
        if T >= len(time):
            T = len(time) - 1
        x = q[0]
        y = q[1]
        theta = q[2]
        e_x = -x_d[T] + x
        e_y = -y_d[T] + y
        e_theta = -theta_d[T] + theta
        v, w = self.control(e_x, e_y, e_theta)
        dotx = v * np.cos(theta)
        doty = v * np.sin(theta)
        dottheta = w
        return np.array([dotx, doty, dottheta])


class PolarRegulator_Control(Control):

    def __init__(self, *args):
        self.name = 'PolarRegulator'
        self.k_1 = args[0]
        self.k_2 = args[1]
        self.k_3 = args[2]
        super(PolarRegulator_Control, self).__init__()

    def control_v(self, *args):
        e_rho, e_delta, e_gamma = args[0]
        v = self.k_1 * e_rho * np.cos(e_gamma)
        return v

    def control_w(self, *args):
        e_rho, e_delta, e_gamma = args[0]
        w = self.k_2 * e_gamma + self.k_1 * ((np.sin(e_gamma) * np.cos(e_gamma)) / e_gamma) * (e_gamma + self.k_3 * e_delta)
        return w

    def model(self, q, t, *args):
        rho = q[0]
        delta = q[1]
        gamma = q[2]
        x_d, y_d, theta_d, time = args

        T = np.searchsorted(time, t)
        if T >= len(time):
            T = len(time) - 1

        x, y, theta = utils.from_polar_to_cartesian([rho, delta, gamma])

        e_x = -x_d[T] + x
        e_y = -y_d[T] + y
        e_theta = -theta_d[T] + theta

        e_rho,e_delta,e_gamma = utils.from_cartesian_to_polar([e_x, e_y, e_theta])
        v, w = self.control(e_rho, e_delta, e_gamma)
        dot_rho = -v * np.cos(gamma)
        dot_delta = (v / rho) * np.sin(gamma)
        dot_gamma = (v / rho) * np.sin(gamma) - w
        return np.array([dot_rho, dot_delta, dot_gamma])
