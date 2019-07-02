import socket
import json
import threading
import base64


def main():
    """
    Main function where you can test how VideoCameraServer works
    """
    # Placing imports here so it will be imported only if user want to test algorithm, not when importing
    # Class VideoCameraServer

    from images_processing_class import ImagesProcessing
    import time

    # How long the Class will receive images
    how_long = 10

    image_processing = ImagesProcessing()

    # Starting Thread which receives data from VideoCamera, port od thread's socket must be the same as the port at
    # which data from VideoCamera is redirected, to be sure check where VideoCamera data stream is send in script env.py
    video_camera_server = VideoCameraServer('localhost', 60011)
    video_camera_server.run()

    # Waiting 1 sec to be sure than video_camera_server has received minimum 1 image, because program will crash if
    # video_camera_server doesn't have time to receive an image
    time.sleep(1)

    # Getting image height and width needed to use create_gray_image and create_rgba_image, I am getting these values
    # now so I don't have to do it in a loop which can consume more computing power, I am also assuming that during
    # the process, simulation will send images with the same height and width as the images at the begging of transfer

    image_height = video_camera_server.current_data['height']
    image_width = video_camera_server.current_data['width']

    time_end = time.time() + how_long
    print("Start")

    while time.time() < time_end:

        # First method: getting only unique images - not getting replicates

        # Checking if data in VideoCamera Thread is new or is already used
        if video_camera_server.is_new_data == True:
            # Setting flag that current data is counted as old and already used
            video_camera_server.is_new_data = False

            # Receiving data from VideoCamera Thread
            encoded_image = video_camera_server.get_image()
        # First method: getting only unique images - not getting replicates

            # Crating image as numpy array from image as a binary object and saving that image
            image = image_processing.create_rgb_image(encoded_image, image_height, image_width)
            # image_processing.save_rgb_image_timestamp(image, "obrazy/")

        # Second method: getting image on every iteration of loop, even if received image is the same as the image from
        # previous iteration
        # It is the same as the first method but without 'if' and without setting 'is_new_data' to false


class VideoCameraServer(threading.Thread):
    """
    Class which receives data from VideoCamera by built-in 'socket' interface, decodes it from json, utf-8 and base64
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
        self.current_image = 0
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
                    self.current_image = base64.b64decode(self.current_data['image'])
                    self.is_new_data = True

                except:
                    pass
                    # print('cannot load VideoCamera data')

    def get_image(self):
        """
        Returns the current, stored in memory image as a binary object, you need to decode it.
        :return: prints info to terminal if wanted data is available
        """
        try:
            return self.current_image
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


if __name__ == '__main__':
    main()
