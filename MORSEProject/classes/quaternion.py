import numpy as np


class Quaternion:
    def __init__(self):
        """
        Initializes quaternion [1.0, 0.0, 0.0, 0.0].
        """
        self.w = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def q(self):
        """
        Use this if you want to receive whole quaternion as a list.
        :return: [q0,q1,q2,q3]
        """
        return [self.w, self.x, self.y, self.z]

    def angle_to_quaternion(self, roll, pitch, yaw):
        """
        This method is used to convert rotation from Euler angles into quaternion.
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        :param roll: rotation on x axis
        :param pitch: rotation on y axis
        :param yaw: rotation on z axis
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        self.w = cy * cp * cr + sy * sp * sr
        self.x = cy * cp * sr - sy * sp * cr
        self.y = sy * cp * sr + cy * sp * cr
        self.z = sy * cp * cr - cy * sp * sr

    def quaternion_to_angles(self):
        """
        Used to obtain Euler angles from current quaternion.
        Received angles are in degrees
        :return: roll, pitch, yaw in degrees
        """

        self.normalize()
        # roll
        sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1.0 - 2.0 * (self.x ** 2 + self.y ** 2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch
        sinp = 2.0 * (self.w * self.y - self.z * self.x)
        if np.fabs(sinp) >= 1.0:
            pitch = np.sign(sinp) * np.pi / 2
        else:
            pitch = np.arcsin(sinp)

        # yaw
        siny_cosp = 2.0 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1.0 - 2.0 * (self.y ** 2 + self.z ** 2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # return roll, pitch, yaw
        return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)

    def invert(self):
        """
        It inverts this instance's quaternion.
        :return: inverted normalized quaternion of this instance
        """
        invQ = Quaternion()
        invQ.w = self.w
        invQ.x = -self.x
        invQ.y = -self.y
        invQ.z = -self.z

        return invQ

    def normalize(self):
        """
        Normalizes quaternion.
        """
        # wanted to use  Fast_inverse_square_root but it's python so let's take it slow
        invSqrt = (self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2) ** -0.5
        self.w = self.w * invSqrt
        self.x = self.x * invSqrt
        self.y = self.y * invSqrt
        self.z = self.z * invSqrt

    @staticmethod
    def multiply_quaternions(q1, q2):
        """
        Calculates quaternion multiplication product - works as multiplying rotation matrices.
        http://www.utdallas.edu/~sxb027100/dock/quaternion.html
        :param q1: first quaternion - needs to be Quaternion type
        :param q2: second quaternion - needs to be Quaternion type
        :return q: multiplied q1*q2
        """
        q = Quaternion()
        q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w

        return q

    def multiply_vec(self, vec):
        """
        Can be useful to multiply current quaternion by some point or other stuff.
        :param vec: input list of length 3
        :return: vector multiplied by quaternion
        """
        num = self.x * 2.0
        num2 = self.y * 2.0
        num3 = self.z * 2.0
        num4 = self.x * num
        num5 = self.y * num2
        num6 = self.z * num3
        num7 = self.x * num2
        num8 = self.x * num3
        num9 = self.y * num3
        num10 = self.w * num
        num11 = self.w * num2
        num12 = self.w * num3

        result = np.zeros(3)
        result[0] = (1.0 - (num5 + num6)) * vec[0] + (num7 - num12) * vec[1] + (num8 + num11) * vec[2]
        result[1] = (num7 + num12) * vec[0] + (1.0 - (num4 + num6)) * vec[1] + (num9 - num10) * vec[2]
        result[2] = (num8 - num11) * vec[0] + (num9 + num10) * vec[1] + (1.0 - (num4 + num5)) * vec[2]

        return result
