import struct
import scipy
import numpy as np
from classes.depth_camera_server import DepthCameraServer
from classes.sensors_classes import Lidar_server, Pose_server
from classes.odometry import *
from math import sqrt
import cv2

"""
Main class that returns 3D map based on depth camera and pose sensor data.
:return:
"""


class DepthCameraPose:

    def __init__(self, ax2D, ax3D):
        # starting position of a robot
        self.robot_pos = [0, 0, 0]

        self.depth_camera_server = DepthCameraServer('localhost', 60012)
        self.depth_camera_server.run()

        self.lidar_server = Lidar_server('localhost', 60008)
        self.lidar_server.run()

        self.pose_server = Pose_server('localhost', 60007)
        self.pose_server.run()

        time.sleep(1)

        self.ax2 = ax2D
        self.ax = ax3D

    def set_position(self, position):
        """
        Set the most actual position after sensor fusion
        :param position: [x, y, z]
        :return:
        """
        self.robot_pos = position

    def rotation(self, x, y, z, yaw, pitch, roll, cam_rotate):
        """
        Function that rotates about three orthogonal axes. These rotations will be referred to as yaw, pitch, and roll.
        :param
            x(float): x coordinate of the sensor, in sensor coordinate, in meter
            y(float): y coordinate of the sensor, in sensor coordinate, in meter
            z(float): z coordinate of the sensor, in sensor coordinate, in meter
            yaw(float): rotation around the Z axis of the sensor, in radian
            pitch(float): rotation around the Y axis of the sensor, in radian
            roll(float): rotation around the X axis of the sensor, in radian
        :return:
            xyz[0](float): x coordinate of the sensor, in world coordinate, in meter
            xyz[1](float): y coordinate of the sensor, in world coordinate, in meter
            xyz[2](float): z coordinate of the sensor, in world coordinate, in meter
        """
        xyz = scipy.array([x, y, z])
        pitch -= cam_rotate
        yaw_cos = math.cos(yaw)
        yaw_sin = math.sin(yaw)
        pitch_cos = math.cos(pitch)
        pitch_sin = math.sin(pitch)
        roll_cos = math.cos(roll)
        roll_sin = math.sin(roll)
        r1 = [yaw_cos * pitch_cos, yaw_cos * pitch_sin * roll_sin - yaw_sin * roll_cos,
              yaw_cos * pitch_sin * roll_cos + yaw_sin * roll_sin]
        r2 = [yaw_sin * pitch_cos, yaw_sin * pitch_sin * roll_sin + yaw_cos * roll_cos,
              yaw_sin * pitch_sin * roll_cos - yaw_cos * roll_sin]
        r3 = [-pitch_sin, pitch_cos * roll_sin, pitch_cos * roll_cos]
        r = scipy.array([r1, r2, r3])
        xyz = np.dot(r, xyz)
        return xyz[0], xyz[1], xyz[2]

    def get_data(self):
        """
        Function that collects data from odometry sensor and depth camera.
        :return:
            points(bytes): decoded 3D point cloud from depth camera server
            pose_x(float): x coordinate of the sensor, in sensor coordinate, in meter
            pose_y(float): y coordinate of the sensor, in sensor coordinate, in meter
            pose_z(float): z coordinate of the sensor, in sensor coordinate, in meter
            yaw(float): rotation around the Z axis of the sensor, in radian
            pitch(float): rotation around the Y axis of the sensor, in radian
            roll(float): rotation around the X axis of the sensor, in radian
        """
        # data received from depth camera coded in base64 format
        points = self.depth_camera_server.get_points()
        # decode data from depth camera base64 format to bytes
        pose_stream = self.pose_server.get_all()
        pose_x = pose_stream['x']
        pose_y = pose_stream["y"]
        pose_z = pose_stream["z"]
        yaw = round(pose_stream['yaw'], 1)
        pitch = round(pose_stream['pitch'], 1)
        roll = round(pose_stream['roll'], 1)
        return points, pose_x, pose_y, pose_z, yaw, pitch, roll

    def get_lidar_data(self):
        """
        Function that collects data from lidar sensor connected to robot.
        :return:
            data_lidar_list(float list): x, y, z coordinates of obstacles, in sensor coordinate, in meter
        """
        data_dict = self.lidar_server.get_all()
        data_lidar_list = []
        for key, value in data_dict.items():
            if key == 'point_list':
                data_lidar_list.extend(value)
        return data_lidar_list

    def points_cloud_update(self, rgb_image):
        """
        Function that converts points collected from depth camera to global coordinates and saves
        them to self.list_of_points.
        :param pos: filtered position [x, y, z]
        """
        # Here we can determine for how long simulation has to collect points to plot them on map
        points, pose_x, pose_y, pose_z, yaw, pitch, roll = self.get_data()

        x, y, z = [], [], []
        ob_x, ob_y, ob_z = [], [], []
        for i in range(0, len(points) - 12, 12):
            # decode coordinates from bytes to float
            xyz = struct.unpack('fff', points[i:i + 12])
            # rotation is included
            x1p, y1p, z1p = self.rotation(xyz[2], xyz[0], xyz[1], yaw, pitch, roll, math.pi/8)
            # data from pose is included, translating
            xp = round(x1p + pose_x, 1)
            yp = round(y1p + pose_y, 1)
            zp = round(z1p + pose_z, 1)
            x.append(xp)
            y.append(yp)
            z.append(zp)
        data_lidar_list = self.get_lidar_data()
        for i in range(len(data_lidar_list) - 1):  # loop for each beam in lidar sensor
            lidar_x = data_lidar_list[i][0]
            lidar_y = data_lidar_list[i][1]
            lidar_z = data_lidar_list[i][2]
            if lidar_x != 0 and lidar_y != 0:
                x1, y1, z1 = self.rotation(lidar_x, lidar_y, lidar_z, yaw, pitch, roll, 0)
                x_l = round(pose_x + x1, 1)
                y_l = round(pose_y + y1, 1)
                z_l = round(pose_z + z1, 1)
                if x_l not in ob_x and y_l not in ob_y:
                    ob_x.append(x_l)
                    ob_y.append(y_l)
                    ob_z.append(z_l)

        inx_dcam, inx_lidar = self.matching(ob_x, ob_y, ob_z, x, y, z)
        x, y = self.shifting(inx_dcam, inx_lidar, ob_x, ob_y, x, y)

        try:
            # Creating array withh rgb colors used to color scatter plot
            colors_array = self.create_color_vector(rgb_image, len(x))
            self.ax.scatter(x, y, z, marker='o', facecolors=colors_array)
            self.ax.scatter(pose_x, pose_y, pose_z, c='r', marker='o')
            self.ax2.plot(x, y, "b.")
            self.ax2.plot(pose_x, pose_y, "r.")
        except:
            # plotting without colouring
            self.ax.scatter(x, y, z, marker='o', c='gray')
            self.ax.scatter(pose_x, pose_y, pose_z, c='r', marker='o')
            self.ax2.plot(x, y, "b.")
            self.ax2.plot(pose_x, pose_y, "r.")

    def matching(self, ob_x, ob_y, ob_z, x, y, z):
        """
        Function that searching for matching points from the depth camera and lidar sensor.
        :return:
            inx_cam(int list): list od indexes of matching points in depth camera points list
           inx_lidar(int list): list of indexes of matching points in lidar sensor points list
        """
        inx_dcam, inx_lidar = [], []
        acc = 0.3  # accuracy
        for i in range(len(x)):
            for j in range(len(ob_x)):
                if abs(x[i] - ob_x[j]) < acc and abs(y[i] - ob_y[j]) < acc and abs(
                        z[i] - ob_z[j]) < acc:
                    inx_dcam.append(i)
                    inx_lidar.append(j)

        return inx_dcam, inx_lidar

    def average(self, lst):
        """
        Function that returns average value of points in list.
        :param lst: list of values of error between lidar and depth camera points.
        :return: average value of points in list.
        """
        try:
            return sum(lst) / len(lst)
        except:
            return 0

    def shifting(self, inx_dcam, inx_lidar, ob_x, ob_y, x, y):
        """
        Function that shifting depth camera points to correcting error.
        :param inx_dcam: (int list) list od indexes of matching points in depth camera points list
        :param inx_lidar: (int list) list of indexes of matching points in lidar sensor points list
        """
        diff_x, diff_y = [], []
        for i in range(len(inx_dcam)):
            x_cam = x[inx_dcam[i]]
            x_lid = ob_x[inx_lidar[i]]
            y_cam = y[inx_dcam[i]]
            y_lid = ob_y[inx_lidar[i]]
            diff_x.append(x_lid - x_cam)
            diff_y.append(y_lid - y_cam)
        aver_diff_x = self.average(diff_x)
        aver_diff_y = self.average(diff_y)
        for i in range(len(x)):
            x[i] += aver_diff_x
            y[i] += aver_diff_y

        return x, y

    def create_color_vector(self, rgb_image, how_many_3d_points):
        """
        Function that returns color vector used to color scatter plot of 3D points, color vector is extracted from
        rgb image and its size depends on amount of points added to plot in current iteration of plotting live
        :param rgb_image: 3 dimensional RGB image from which colors will be extracted
        :param how_many_3d_points: amount of points received from Depth Camera in current iteration
        :return: numpy array with size (how_many_3d_points, 3) storing RGB colors for each 3D points from current
        iteration of plotting live
        """
        dimension = int(sqrt(how_many_3d_points))  # Calculating approximate size of new image with size more
        # compatible with 3D points

        resized_image = cv2.resize(rgb_image, (dimension, dimension))  # Creating new image with size more compatible
        # with 3D points

        colors_array = np.reshape(resized_image, (dimension * dimension,
                                                  3))  # Creating 2 dimensional vector from 3 dimensional image, so
        # the shape of this vector is more similar to shape of list with 3D points and consecutive RGB values of
        # pixels are corresponding to consecutive points in cloud of 3D points.

        colors_array = colors_array / 255  # Rescaling values of pixels from 0-255 to 0-1

        # Resizing vector with values of colors so the shape of it is exactly the same as shape of list with 3D points,
        # missing elements in new vector are fulfilled with copies of original vector
        colors_array = np.resize(colors_array, (how_many_3d_points, 3))

        return colors_array