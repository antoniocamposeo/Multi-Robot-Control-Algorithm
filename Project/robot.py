import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import Project.control as control

control_name = ['TrajectoryTracking', 'IOLinearization', 'CartesianRegulator', 'PolarRegulator']


class Robot:
    def __init__(self, name):
        self.robot_to_follow = None
        self.control = None
        self.name = name
        self.q0 = None
        self.control_obj = None
        self.robot_states = None
        self.path = None
        self.x_d = None
        self.y_d = None
        self.doty_d = None
        self.dotx_d = None
        self.t = None
        self.gains = None
        self.sim_args = None

    def set_robot_to_follow(self, robot):
        self.robot_to_follow = robot

    def get_robot_to_follow(self):
        return self.robot_to_follow

    def set_sim_args(self, *args):
        self.sim_args = args

    def get_sim_args(self):
        return self.sim_args

    def set_control_obj(self, *args):
        if self.control == control_name[0] and args is None:
            self.control_obj = control.TrajectoryTracking_Control(self.gains)
        elif self.control == control_name[1] and args is None:
            self.control_obj = control.I_O_Linearization_Control(self.gains)
        elif self.control == control_name[2] and args is None:
            self.control_obj = control.CartesianRegulator_Control(self.gains)
        elif self.control == control_name[3] and args is None:
            self.control_obj = control.PolarRegulator_Control(self.gains)
        elif len(args) == 1:
            self.control_obj = args[0]
            self.control = self.control_obj.name

    def set_time(self, t):
        self.t = t

    def get_time(self):
        return self.t

    def set_path(self, path):
        self.path = path

    def get_path(self):
        return self.path

    def set_initial_state(self, q0):
        self.q0 = q0

    def get_initial_state(self):
        return self.q0

    def get_x(self):
        return self.robot_states[:, 0]

    def get_y(self):
        return self.robot_states[:, 1]

    def get_theta(self):
        return self.robot_states[:, 2]

    def get_control_obj(self):
        return self.control_obj

    def simulation(self):
        if self.q0 is None or self.t is None or self.control_obj is None:
            print('Error: the intial conditions are not set.')
            raise ValueError
        if self.control == control_name[0]:
            self.sim_args = (
                self.path[0]['x'], self.path[0]['y'], self.path[0]['theta'], self.path[0]['v'], self.path[0]['w'],
                self.t)
        elif self.control == control_name[1]:
            self.sim_args = (self.path[0]['x'], self.path[0]['y'], self.path[0]['dxdt'], self.path[0]['dydt'], self.t)
        elif self.control == control_name[2]:
            self.sim_args = (self.path[0]['x'], self.path[0]['y'], self.path[0]['theta'], self.t)
        elif self.control == control_name[3]:
            self.sim_args = (self.path[0]['x'], self.path[0]['y'], self.path[0]['theta'], self.t)

        if self.sim_args is None:
            self.robot_states = odeint(self.control_obj.model, self.q0, self.t)
        else:
            self.robot_states = odeint(self.control_obj.model, self.q0, self.t, self.sim_args)

        if self.control == control_name[3]:
            x = [q[0] * np.cos(q[1] + np.pi) for q in self.robot_states]
            y = [q[0] * np.sin(q[1] + np.pi) for q in self.robot_states]
            theta = [np.arctan2(np.sin(q[1] - q[2]), np.cos(q[1] - q[2])) for q in self.robot_states]
            self.robot_states[:, 0] = x
            self.robot_states[:, 1] = y
            self.robot_states[:, 2] = theta

    def reset_states(self):
        self.robot_states = None
        self.control_obj = None
        self.sim_args = None
        self.gains = None