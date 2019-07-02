import numpy as np


class Kalman3D:
    """
    Class that helps estimate current position based on observations.
    You need to run update method for calculations to make place

    """
    def __init__(self, dt=1/60.0):
        """
        Initialize all used matrices.
        :param dt: 1 / frequency at wchich sensors work, default value is 60Hz
        """
        self.gain = np.eye(6) * 0.25  # g = Eest/(Eset+Emea) <0,1>
        self.dt = dt  # time between samples

        self.state = np.array([0, 0, 0, 0, 0, 0])  # state matrix with initial state (x,y positions and x,y velocities)

        # accelerations on input
        self.B = np.array([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [self.dt, 0, 0],
                           [0, self.dt, 0],
                           [0, 0, self.dt]])  # input vector, maby do not take z under account because of gravity

        self.G = np.array([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])

        # positions then accelerations
        self.A = np.array([[1, 0, 0, self.dt, 0, 0],
                            [0, 1, 0, 0, self.dt, 0],
                            [0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])

        # these values will figure out themselves during iterations
        self.P = np.array([[50, 0, 0, 0, 0, 0],
                            [0, 50, 0, 0, 0, 0],
                            [0, 0, 50, 0, 0, 0],
                            [0, 0, 0, 50, 0, 0],
                            [0, 0, 0, 0, 50, 0],
                            [0, 0, 0, 0, 0, 50]])  # state covariant matrix(error in the estimate)

        # it's hard to figure out -> issue #38
        self.s_d_q = 0.05
        # process noise covariance matrix
        self.Q = np.array([[self.s_d_q**2, 0, 0],
                           [0, self.s_d_q**2, 0],
                           [0, 0, self.s_d_q**2]])  # state covariant matrix(error in the estimate)

        # this also need's to be figured out but only when there will be position from "odometry"
        self.s_d_r = 0.25
        self.R = np.array([[self.s_d_r**2, 0, 0],
                            [0, self.s_d_r**2, 0],
                            [0, 0, self.s_d_r**2]])  # measurement covariance matrix(error in the measurement)

        # matrix that helps to get only values needed from different size matrix
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0]])

        self.I = np.eye(6)
        self.prediction = np.array([0, 0, 0])

    def get_predicted_position(self):
        """
        :return: [x,y,z] last predicted position
        """
        return self.prediction

    def get_estimated_position(self):
        """
        :return: [x,y,z] estimated current position based on last given observation to update
        """
        return np.matmul(self.H, self.state)

    def update(self, measured_position, accelerations):
        """
        Updates kalman with position and accelerations
        :param measured_position: [x,y,z] position from odometry or other position sensor
        :param accelerations: [ax,ay,az] in m^2/s
        :return:
        """

        accelerations[2] = 0  # just because gravity sucks

        # predict new state
        self.state = np.matmul(self.A, self.state) + np.matmul(self.B, accelerations)

        self.P = np.matmul(np.matmul(self.A, self.P), np.transpose(self.A))\
                 + np.matmul(np.matmul(self.G, self.Q), np.transpose(self.G))

        # get only position that we can observe
        self.prediction = np.matmul(self.H, self.state)
        innovation = measured_position - self.prediction

        # new gain matrix
        innovation_covariance = np.matmul(np.matmul(self.H, self.P), np.transpose(self.H)) + self.R
        self.gain = np.matmul(np.matmul(self.P, np.transpose(self.H)), np.linalg.inv(innovation_covariance))

        # estimate current state
        self.state = self.state + np.matmul(self.gain, innovation)

        # update prediction matrix
        self.P = np.matmul(self.I - np.matmul(self.gain, self.H), self.P)
