import socket
import json
import threading


def main():

    import time

    how_long = 10   # How long data from sensors will be received

    pose_server = Pose_server('localhost', 60007)
    pose_server.run()

    lidar_server = Lidar_server('localhost', 60008)
    lidar_server.run()

    time.sleep(1)   # Waiting for sensors to collect first data

    time_end = time.time() + how_long

    while time.time() < time_end:

        pose_data = pose_server.get_all()
        lidar_data = lidar_server.get_all()


class Lidar_server(threading.Thread):
    """
    Class which receives data from Lidar by built-in 'socket' interface, decodes it from json and makes it available
    for user
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

        self.current_data = ''
        self.is_new_data = False

    def recv_end(self):
        """
        This method is used to obtain only one json object from socket.
        Sometimes it still can return not valid format if it starts receiving in the middle of transmission.
        :return: string with json formatting
        """
        total_data = []

        while True:
            data = self.sock.recv(512)
            data = data.decode('utf-8')

            # If there is endline in message, it means this is the end of this message
            if self.endline in data:
                total_data.append(data[:data.find(self.endline)])
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
                    self.is_new_data = True

                except:
                    pass

    def get(self, arg):
        """
        Returns the value of given argument from the stream if such exists.
        :param arg: name of the parameter in received json data, data is in form of dict so you have to pass key.
        :return: prints info to terminal if wanted data is available
        """
        try:
            val = self.current_data[arg]
            return val
        except:
            pass

    def get_all(self):
        """
        Return the whole sample of data from stream (data is in form of python dict)
        :return: python dict with data from sensor
        """
        try:
            return self.current_data
        except:
            pass

    def run(self):
        """
        Runs the receiving method as thread.
        :return:
        """
        receiver = threading.Thread(target=self.receive_data)
        # Setting daemon to True means that this Thread will be terminated when the main program ends.
        receiver.daemon = True
        receiver.start()


class Pose_server(threading.Thread):
    """
    Class which receives data from Lidar by built-in 'socket' interface, decodes it from json and makes it available
    for user
    """

    def __init__(self, host, port):
        """
        Initializes socket and opens data stream.
        :param host: address of a streaming socket
        :param port: port on which is the stream
        """
        # super(Pose_server, self).__init__()
        self.endline = '\n'
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.sock.connect((host, port))

        self.current_data = ''
        self.is_new_data = False

    def recv_end(self):
        """
        This method is used to obtain only one json object from socket.
        Sometimes it still can return not valid format if it starts receiving in the middle of transmission.
        :return: string with json formatting
        """
        total_data = []

        while True:
            data = self.sock.recv(512)
            data = data.decode('utf-8')

            # If there is endline in message, it means this is the end of this message
            if self.endline in data:
                total_data.append(data[:data.find(self.endline)])
                break

            total_data.append(data)

            if len(total_data) > 1:
                # check if end_of_data was split
                last_pair=total_data[-2]+total_data[-1]

                if self.endline in last_pair:
                    total_data[-2]= last_pair[:last_pair.find(self.endline)]
                    total_data.pop()
                    break

        return ''.join(total_data)

    def receive_data(self):
        """
        Keeps the actual reader data up-to-date. Should be run as thread.
        :return: prints info to terminal if received data is not valid
        When full data sample is received and assembled, flag is_new_data is set to True
        """
        while True:

            data = self.recv_end()

            if len(data) > 0:

                try:
                    self.current_data = json.loads(data)
                    self.is_new_data = True
                except:
                    pass

    def get(self, arg):
        """
        Returns the value of given argument from the stream if such exists.
        :param arg: name of the parameter in received json data, data is in form of dict so you have to pass key.
        :return: prints info to terminal if wanted data is available
        """
        try:
            val = self.current_data[arg]
            return val
        except:
            pass

    def get_all(self):
        """
        Return the whole sample of data from stream (data is in form of python dict)
        :return: python dict with data from sensor
        """

        try:
            return self.current_data
        except:
            pass

    def run(self):
        """
        Runs the receiving method as thread.
        :return:
        """
        receiver = threading.Thread(target=self.receive_data)
        # Setting daemon to True means that this Thread will be terminated when the main program ends.
        receiver.daemon = True
        receiver.start()


if __name__ == '__main__':
    main()
