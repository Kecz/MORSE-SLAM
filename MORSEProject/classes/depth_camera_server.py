import socket
import json
import threading
import base64


def main():
    """
    Main function where you can test how VideoCameraServer works
    """
    # Placing imports here so it will be imported only if user want to test algorithm, not when importing
    # Class DepthCameraServer

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import sensors_classes as sensors
    from images_processing_class import ImagesProcessing
    import struct
    import time

    # Starting Thread which receives data from VideoCamera, port od thread's socket must be the same as the port at
    # which data from VideoCamera is redirected, to be sure check where VideoCamera data stream is send in script env.py
    depth_camera_server = DepthCameraServer('localhost', 60012)
    depth_camera_server.run()

    pose_server = sensors.Pose_server('localhost', 60007)
    pose_server.run()

    # Waiting 1 sec to be sure than depth_camera_server has received minimum 1 image, because program will crash if
    # depth_camera_server doesn't have time to receive an image
    time.sleep(1)

    points = depth_camera_server.get_points()

    lista_punktow = []
    x = []
    y = []
    z = []

    data_pose_dict = pose_server.get_all()
    pose_x = data_pose_dict['x']
    pose_y = data_pose_dict['y']
    pose_z = data_pose_dict['z']

    yawp = data_pose_dict['yaw']
    pitchp = data_pose_dict['pitch']
    rollp = data_pose_dict['roll']

    # Each 3D point is a set of float(x,y,z). Each point has a size of 12 bytes because
    # 3*sizeof(float) = 12 bytes, that's why we are dividing data into parts with size of 12 and then
    # converting this data to tuple with 3 float (xyz).

    #
    # Processing cloud of points to seperate x, y and z was copied from dcam_old.py
    #

    for i in range(0, len(points) - 12, 12):
        xyz = struct.unpack('fff', points[i:i + 12])

        # rotation is included
        x1p, y1p, z1p = rotation(xyz[2], xyz[0], xyz[1], yawp, pitchp, rollp)

        # data from pose is included
        xp = round(x1p + pose_x, 1)
        yp = round(y1p + pose_y, 1)
        zp = round(z1p + pose_z, 1)
        temp = [xp, yp, zp]
        lista_punktow.append(temp)

    # Choosing only these points which have minimum 0.45 meters at z-axis, but why???
    for i in lista_punktow:
        x.append(i[0])
        y.append(i[1])
        z.append(i[2])

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(x, y, z, cmap='viridis', linewidth=0.5)
    ax.scatter(x[0], y[0], z[0], c='red')
    ax.scatter(x[1], y[1], z[1], c='yellow')
    ax.scatter(x[2], y[2], z[2], c='black')
    ax.scatter(pose_x, pose_y, pose_z, c='green')
    plt.show()


class DepthCameraServer(threading.Thread):
    """
    Class which receives data from DepthCamera by built-in 'socket' interface, decodes it from json, utf-8 and base64
    and makes it available for user
    """

    def __init__(self, host, port):
        """
        Initializes socket and opens data stream.
        :param host: address of a streaming socket
        :param port: port on which is the stream
        """
        self.endline = '\n'
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.sock.connect((host, port))

        self.current_data = 0
        self.current_3D_points = 0
        self.is_new_data = False

    def recv_end(self):
        """
        This method is used to obtain only one object from socket.
        Sometimes it still can return not valid format if it starts receiving in the middle of transmission.
        :return: string with json formatting
        """
        total_data = []

        while True:
            data = self.sock.recv(65536)
            data = data.decode('utf-8')

            # If there is endline in message, it means this is the end of this message
            if self.endline in data:
                single_data = data[:data.find(self.endline)]
                total_data.append(single_data)
                break

            total_data.append(data)

            if len(total_data) > 1:
                # check if end_of_data was split
                last_pair = total_data[-2]+total_data[-1]
                if self.endline in last_pair:
                    total_data[-2] = last_pair[:last_pair.find(self.endline)]
                    total_data.pop()
                    break

        return ''.join(total_data)

    def receive_data(self):
        """
        Keeps the actual sensor data up-to-date. Should be run as thread.
        It also prints info to terminal if received data is not valid.
        When full data sample is received and put together, flag is_new_data is set to True
        """
        while True:

            data = self.recv_end()

            if len(data) > 0:

                try:
                    self.current_data = json.loads(data)
                    self.current_3D_points = base64.b64decode(self.current_data['points'])
                    self.is_new_data = True

                except:
                    # print('cannot load DepthCamera data')
                    pass

    def get_points(self):
        """
        Returns the cloud of 3D points as a bytes object, decoding is needed
        :return: prints info to terminal if wanted data is available
        """
        try:
            return self.current_3D_points
        except:
            print('no such current_image')

    def get_all(self):
        """
        Return the whole sample of data from stream (data is in form of python dict)
        :return: python dict with data from sensor
        """
        try:
            return self.current_data
        except:
            print('No data received from sensor')

    def run(self):
        """
        Runs the receiving method as thread.
        :return:
        """
        receiver = threading.Thread(target=self.receive_data)
        # Setting daemon to True means that this Thread will be terminated when the main program ends.
        receiver.daemon = True
        receiver.start()


# function rotating view from DepthCamera
def rotation(x, y, z, yaw, pitch, roll):
    import math
    import scipy
    import numpy as np
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
    camera_translate = - math.pi / 8
    pitch += camera_translate
    xyz = scipy.array([x, y, z])
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


if __name__ == '__main__':
    main()
