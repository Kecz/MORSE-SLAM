from classes.dataStreamReader import *
import time
import math
from classes.sensors_classes import Pose_server


"""
Main class that returns position of robot based on differential odometry sensor data.
:return:
"""

class Odometry():

    def __init__(self):
        #list of positions from odometry sensor
        # self.new_pos = []
        #list of positions from pose sensor
        # self.pose = []
        # create reader to obtain live data from pose
        # self.pose_data = Reader('localhost', 60007)
        # self.pose_data.start()
        # create reader to obtain live data from odometry
        #self.odometry_data = Reader('localhost', 60006)
        #self.odometry_data.start()
        self.odometry_data = Pose_server('localhost', 60006)    # Na porcie 60006 jest Odometr, ale po prostu używany jest
                                                            # ten sam serwer co dla Pose bo to ten sam format i typ danych
        self.odometry_data.run()
        # readers need a little warmup
        time.sleep(0.5)

        # x = self.pose_data.get('x')
        # y = self.pose_data.get('y')
        # z = self.pose_data.get('z')
        # yaw = self.pose_data.get('yaw')
        # pitch = self.pose_data.get('pitch')
        # roll = self.pose_data.get('roll')

        # first element is data from pose
        # self.new_pos.append([x, y, z, yaw, pitch, roll])
        # self.pose.append([x, y, z, yaw, pitch, roll])
        # self.new_pos.append([0, 0, 0, 0, 0, 0])
        # self.pose.append([0, 0, 0, 0, 0, 0])
        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0

    def normalise(self,ang):
        """
        Function that returns angle between (0,360) degrees
        :param ang: angle to normalise
        :return:
            ang: normalised angle
        """
        while(ang>math.pi):
            ang+=-2*math.pi
        while ang <-math.pi:
            ang+=2*math.pi
        return ang

    def get_data(self):
        """
        Function that get data from odometry sensor and compute position of robot.
        :return:
            x(float): x coordinate of robot position
            y(float): y coordinate of robot position
            z(float): z coordinate of robot position
            yaw(float): rotation around the Z axis
            pitch(float): rotation around the Y axis
            roll(float): rotation around the X axis

        """
        # time.sleep(0.03)
        #Leave reading pose data in case of wanting compare postions

        # self.x = self.pose_data.get('x')
        # self.y =self.pose_data.get('y')
        # self.z=self.pose_data.get('z')
        # self.yaw=self.pose_data.get('yaw')
        # self.pitch=self.pose_data.get('pitch')
        # self.roll=self.pose_data.get('roll')

        all = self.odometry_data.get_all()
        o_dx = all['dx']
        o_dy = all['dy']
        o_dz = all['dz']
        o_yaw = all['dyaw']
        o_pitch = all['dpitch']
        o_roll = all['droll']

        self.x += o_dx
        self.y += o_dy
        self.z += o_dz

        self.roll += o_roll
        self.pitch += o_pitch
        self.yaw += o_yaw

        return self.x, self.y, self.z, self.yaw, self.pitch, self.roll

        # self.new_pos.append([self.new_pos[-1][0]+o_dx,self.new_pos[-1][1]+o_dy,self.new_pos[-1][2]+o_dz,self.normalise(o_yaw+self.new_pos[-1][3]),self.normalise(o_pitch+self.new_pos[-1][4]),self.normalise(o_roll+self.new_pos[-1][5])])
        # #print(self.new_pos[-1])
        # #print(self.pose[-1])
        #
        # return self.new_pos[-1][0],self.new_pos[-1][1],self.new_pos[-1][2],self.new_pos[-1][3],self.new_pos[-1][4],self.new_pos[-1][5]