import socket
import json
import threading

# IMU data: timestamp, angular_velocity, linear_acceleration, magnetic_field
# Lidar data: timestamp, point_list, range_list
# Pose data: timestamp, x, y, z, yaw, pitch, roll


class Reader(threading.Thread):
    End = '\n'

    def __init__(self, host='localhost', port=60000):
        """
        Initializes socket and opens data stream.
        :param host: address of a streaming socket
        :param port: port on which is the stream
        """

        super(Reader, self).__init__()
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.sock.connect((host, port))

        self.data = ''

    def recv_end(self):
        """
        This method is used to obtain only one json object from socket.
        Sometimes it still can return not valid format if it starts receiving in the middle of transmission.
        :return: string with json formatting
        """
        total_data=[]
        while True:
                data=self.sock.recv(512)
                data = data.decode('utf-8')
                if Reader.End in data:
                    total_data.append(data[:data.find(Reader.End)])
                    break
                total_data.append(data)
                if len(total_data)>1:
                    # check if end_of_data was split
                    last_pair=total_data[-2]+total_data[-1]
                    if Reader.End in last_pair:
                        total_data[-2]=last_pair[:last_pair.find(Reader.End)]
                        total_data.pop()
                        break
        return ''.join(total_data)

    def receive_data(self):
        """
        Keeps the actual reader data up-to-date. Should be run as thread.
        :return: prints info to terminal if received data is not valid
        """
        while (True):
            # data = self.sock.recv(4096)
            data = self.recv_end()
            # if data received
            if len(data) > 0:
                # print(data.decode('utf-8'))
                try:
                    self.data = json.loads(data)
                except:
                    pass
                    # print('cannot load data')

    def get(self, arg):
        """
        Returns the value of given argument from the stream if such exists.
        :param arg: name of the parameter in received json data
        :return: prints info to terminal if wanted data is available
        """
        try:
            val = self.data[arg]
            return val
        except:
            pass
            # print('no such data')

    def get_all(self):
        """
        Return the whole sample of data from stream (data is in form of python dict)
        :return: python dict with data from sensor
        """

        try:
            return self.data
        except:
            pass

    def run(self):
        """
        Runs the receiving method as thread.
        :return:
        """
        receiver = threading.Thread(target=self.receive_data)
        receiver.start()
