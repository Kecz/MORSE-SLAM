"""
Test szybkości dla rozdzielczości 600x600:

Odczyt, tworzenie i zapis zdjęć (z duplikatami czyli bez tego 'if is_new_data == True'):
Podczas 20 sekund udało się zapisać 455 zdjęć RGB albo 1200 zdjęć w grayscale 1 channel albo 546 zdjęć w grayscale
3 channels.

Tylko odczyt i tworzenie zdjęć, bez ich zapisywania:
Podczas 20 sekund udało się utworzyć 57063568 zdjęć RGB (wszystkie nowe, bez wielokrotnego zapisywania tego samego
zdjęcia) albo miliony zdjęć grayscale albo miliony zdjęć w grayscale 3 channels

Wniosek z tego taki, że samo tworzenie obrazów idzie teraz bardzo szybko, to zapis jest czasochłonny i też to
Video Camera głównie opóżnia, mam na myśli, że obiekt klasy ImagesProcessing szybciej pobiera obrazy z obiektu klasy
VideoCameraServer i je zapisuje niż VideoCameraServer odbiera nowe obrazy z Video Camery, dekoduje je i wystawia jako
nowe zdjęcia.

Jeśli będzie się chciało zapisywać tylko nowe obrazy to wtedy szybkośc drastycznie zmaleje, nawet kilkudziesięciokrotnie,
wtedy pewnie potrzebne będzie zmiejszenie rozdzielczości Video Camery na mniejszą niż 600x600.
"""
import matplotlib.pyplot as plt
import numpy as np
import scipy.misc
import argparse
import time
import cv2
from math import sqrt
# from PIL import Image

how_long = 20
default_path = 'obrazy/'

do_save_rgb = 1
do_save_gray_1channel = 0
do_save_gray_3channel = 0


def main():  # main for testing and showing how to use
    """
    Function that connects to the MORSE simulation and allows us to access data from camera and process it using
    prepared functions or our own algorithms
    """

    from video_camera_server import VideoCameraServer

    print("Accessing data... ")

    parser = parse_args()

    image_processing = ImagesProcessing()

    video_camera_server = VideoCameraServer('localhost', 60011)
    video_camera_server.run()
    time.sleep(0.5)

    # Checking camera properties to read size of image sent by camera
    camera_properties = video_camera_server.get_all()
    img_width = camera_properties['width']
    img_height = camera_properties['height']

    # Loop in which data from camera is downloaded, processed and can be send further
    time_end = time.time() + parser.time
    licznik = 0

    while time.time() < time_end:
        # Receiving one sample of data from camera, data field of image is a base64 encoded RGBA image
        image = video_camera_server.get_image()

        if do_save_rgb:
            # RGB Image
            if video_camera_server.is_new_data == True:
                licznik += 1
                video_camera_server.is_new_data = False
                rgb_image = image_processing.create_rgb_image(image, img_height, img_width)

                # image_processing.plot_rgb_image(rgb_image)

                # Save image to png file
                image_processing.save_rgb_image_absolute_path(rgb_image, default_path+'rgb'+str(licznik).rjust(4, '0')+'.png')
                # image_processing.save_rgb_image_timestamp(image, parser.save)

                # plot_rgb_image(rgb_image)

        if do_save_gray_1channel:

            # Grayscale Image
            if video_camera_server.is_new_data == True:
                video_camera_server.is_new_data = False
                licznik += 1
                gray_image = image_processing.create_gray_image(image, img_height, img_width)

                # Save image to png file
                image_processing.save_grayscale_image_absolute_path(gray_image, default_path+'gray'+str(licznik).rjust(4, '0')+'.png')
                # plot_gray_image(gray_image)

        if do_save_gray_3channel:

            # Grayscale Image
            if video_camera_server.is_new_data == True:
                licznik += 1
                video_camera_server.is_new_data = False

                gray_image = image_processing.create_gray_image_3channels(image, img_height, img_width)

                # Save image to png file
                image_processing.save_grayscale_image_absolute_path(gray_image, default_path+'gray3ch'+str(licznik).rjust(4, '0')+'.png')
                # plot_gray_image(gray_image)

    print(licznik)


def parse_args():
    """
    Function that creates argparse which allows user to pass arguments to script from command-line.

    :return: Parser containing variable 'time' (in seconds) which tells how long the script should receive data from
    camera and process it and variable 'save' with path to folder in which images are saved

    Example of how to pass these arguments:
        python3 images_processing_class.py -time 20 -save 'name_of_folder/'

    """

    parser = argparse.ArgumentParser(description='Input command line arguments for images_processing_class.py script')

    parser.add_argument('-time', metavar="Duration of saving data from camera", type=float,
                        help='For how much time script will download data from camera and process it (in seconds). '
                             'Default: 10 seconds',
                        default=how_long)

    parser.add_argument('-save', metavar="Path to output folder", type=str,
                        help='Path to output folder in which images received from camera are saved. This path includes '
                             'only path to folder because names of images are generated automatically based on '
                             'timestamp and type of image (RGB or Grayscale). Default: "obrazy/" ',
                        default=default_path)

    return parser.parse_args()


class ImagesProcessing:
    """
    Class providing many functions to work on images, for example functions to create RGB or grayscale images from
    images as binary objects and functions to save and plot these images.
    This class can also store one image in case it is needed.
    """
    def __init__(self):
        self.internal_image = None

    def create_rgba_image(self, binary_image, img_height, img_width):
        """
        Function creates RGBA image from image in binary format, values of pixels of the image are from 0 do 255.

        Tips:
            Using matplotlib.pyplot: If you want to show this image on your own using matplotlib.pyplot remember
                that you need to scale pixels' values to <0, 1>.

            Using OpenCV: Pay attention that if you want to show this image using OpenCV, you will get image with reversed
                colors because default format for OpenCV is BGR.

            Using scipy.misc.imsave: To use scipy to save image pixels' value can be either <0, 1> or <0, 255>.


        :param binary_image: image in binary format
        :param img_width: width of image, can be accessed from videocamera parameter 'cam_width'
        :param img_height: height of image, can be accessed from videocamera parameter 'cam_height'

        :return: RGBA image in form of a numpy array with shape (img_height, img_width, 3) and with values of pixels from 0
        to 255

        """

        # Creating RGBA image - first method (the slowest) - 66 images in 20 seconds with saving
        """
        Rather slow: creates 66 images in 20 seconds
        
        RGBA_image = np.empty((img_width * img_height, 3))

        for index in range(0, int(len(binary_image) / 4)):
            RGB_image[index][0] = binary_image[4 * index + 0]
            RGB_image[index][1] = binary_image[4 * index + 1]
            RGB_image[index][2] = binary_image[4 * index + 2]

        RGBA_image = np.reshape(RGBA_image, (img_height, img_width, 3))

        # Changing format of image so it can be used by Open-CV functions and many more
        RGBA_image = RGBA_image.astype(np.uint8)
        """

        # Creating RGBA image - second method (the fastest) - 425-455 images in 20 seconds with saving
        RGBA_image = np.ndarray(shape=(img_width, img_height, 4), buffer=binary_image, dtype='uint8')

        # Creating RGBA image - third method (very fast but a bit slower than the second method) - 370-417 images in 20 seconds with saving
        # RGBA_image = Image.frombuffer('RGBA', (img_width, img_height), image, 'raw', 'RGBA', 0, 1)

        return RGBA_image

    def create_rgb_image(self, binary_image, img_height, img_width):
        """
        Function creates RGB image based on RGBA image created from image in binary format.

        :param binary_image: image in binary format
        :param img_width: width of image, can be accessed from videocamera parameter 'cam_width'
        :param img_height: height of image, can be accessed from videocamera parameter 'cam_height'
        :return: RGB image as numpy array
        """

        # Creating RGB image - 600 images in 20 seconds
        rgba_image = self.create_rgba_image(binary_image, img_height, img_width)
        rgb_image = cv2.cvtColor(rgba_image, cv2.COLOR_RGBA2RGB)

        return rgb_image

    def create_gray_image(self, binary_image, img_height, img_width):
        """
        Function creates 1 channel grayscale image based on RGBA image created from image in binary format.
        Other methods are creating grayscale image from image in binary format using grayscale formula:
        0.2126*R + 0.7152*G + 0.0722*B.
        Values of pixels of the image are from 0 do 255.

        Tips:
            Using matplotlib.pyplot: If you want to plot this image on your own remember that because this is one channel
                image, you have to manually choose colormap, in this case you should use "cmap='gray'" to get grayscale
                image.

        :param binary_image: image in binary format
        :param img_width: width of image, can be accessed from videocamera parameter 'cam_width'
        :param img_height: height of image, can be accessed from videocamera parameter 'cam_height'

        :return: Grayscale image in form of a numpy array with shape (img_height, img_width, 1) and with values of pixels
        from 0 to 255
        """
        # First method - creating grayscale binary_image with list comprehension - saving 127 images in 20 seconds
        """
        gray_image = [int(0.2126 * binary_image[index] + 0.7152 * binary_image[index + 1] + 0.0722 * binary_image[index + 2])
                      for index in range(0, len(binary_image), 4)]

        np_gray_img = np.array(gray_image).reshape(img_height, img_width)
        """

        # Second method - creating RGB image with very fast method and then converting it to grayscale with openCV
        # Very fast - saving 1200 images in 20 seconds with saving
        rgba_image = self.create_rgba_image(binary_image, img_height, img_width)
        np_gray_img = cv2.cvtColor(rgba_image, cv2.COLOR_RGBA2GRAY)

        return np_gray_img

    def create_gray_image_3channels(self, binary_image, img_height, img_width):
        """
        Function creates 3 channel grayscale image based on RGBA image created from image in binary format.
        Other methods are creating grayscale image from image in binary format using grayscale formula:
        0.2126*R + 0.7152*G + 0.0722*B.
        Values of pixels of the image are from 0 do 255.

        :param binary_image: image in binary format
        :param img_width: width of image, can be accessed from videocamera parameter 'cam_width'
        :param img_height: height of image, can be accessed from videocamera parameter 'cam_height'

        :return: Grayscale image in form of a numpy array with shape (img_height, img_width, 3) and with values of pixels
        from 0 to 255
        """
        # First method - manually creating gray scale image with 3 channels - slow - saving 140-150 images in 20 seconds with saving
        """
        # Creating grayscale binary_image with list comprehension
        gray_image = np.array([int(0.2126 * binary_image[index] + 0.7152 * binary_image[index + 1] + 0.0722 * binary_image[index + 2])
                      for index in range(0, len(binary_image), 4)])
        np_gray_img_3channels = np.zeros((img_height, img_width, 3))
        np_gray_img_3channels[:,:,0] = gray_image.reshape(img_height, img_width)
        np_gray_img_3channels[:,:,1] = gray_image.reshape(img_height, img_width)
        np_gray_img_3channels[:,:,2] = gray_image.reshape(img_height, img_width)
        
        """

        # Second method - creating 1 channel grayscale image using very fast method and then dupicating it to create
        # 3 channel grayscale image - fast - saving 546 images in 20 seconds with saving
        rgba_image = self.create_rgba_image(binary_image, img_height, img_width)
        gray_image = cv2.cvtColor(rgba_image, cv2.COLOR_RGBA2GRAY)

        np_gray_img_3channels = np.zeros((img_height, img_width, 3))
        np_gray_img_3channels[:, :, 0] = gray_image.reshape(img_height, img_width)
        np_gray_img_3channels[:, :, 1] = gray_image.reshape(img_height, img_width)
        np_gray_img_3channels[:, :, 2] = gray_image.reshape(img_height, img_width)

        return np_gray_img_3channels

    def plot_rgb_image(self, image):
        """
        Function that plots input RGB image using matplotlib.pyplot.plot.

        :param image: Input RGB image that will be plotted
        """
        image = image / 255
        plt.imshow(image)
        plt.show()

    def plot_gray_image(self, image):
        """
        Function that plots input grayscale image using matplotlib.pyplot.plot.

        :param image: Input grayscale image that will be plotted
        :return:
        """
        image = image / 255
        plt.imshow(image, cmap='gray')
        plt.show()

    def save_rgb_image_timestamp(self, image, path):
        """
        Function that saves RGB image to .png file. You can specify in which folder image will be saved, but image name is
        generated automatically based on timestamp.

        :param image: Input RGB image as a numpy array.
        :param path: Path to folder in which image will be saved. For example 'my_images/'
        """
        image_path = path + 'rgb_image' + str(time.time()) + '.png'
        scipy.misc.imsave(image_path, image)

    def save_rgb_image_absolute_path(self, image, path):
        """
        Function that saves RGB image to .png file with absolute path.

        :param image: Input RGB image as a numpy array.
        :param path: Absolute path where image will be saved
        """
        scipy.misc.imsave(path, image)

    def save_grayscale_image_timestamp(self, image, path):
        """
        Function that saves Grayscale image to .png file. You can specify in which folder image will be saved, but image
        name is generated automatically based on timestamp.

        :param image: Input Grayscale image as a numpy array.
        :param path: Path to folder in which image will be saved. For example 'my_images/'
        """
        image_path = path + 'grayscale_image' + str(time.time()) + '.png'
        scipy.misc.imsave(image_path, image)

    def save_grayscale_image_absolute_path(self, image, path):
        """
        Function that saves Grayscale image to .png file with absolute path.

        :param image: Input Grayscale image as a numpy array.
        :param path: Absolute path where image will be saved
        """
        scipy.misc.imsave(path, image)

    def create_color_vector(self, rgb_image, how_many_3d_points):
        """
        Function that returns color vector used to color scatter plot of 3D points, color vector is extracted from
        rgb image and its size depends on amount of points added to plot in current iteration of plotting live
        :param rgb_image: 3 dimensional RGB image from which colors will be extracted
        :param how_many_3d_points: amount of points received from Depth Camera in current iteration
        :return: numpy array with size (how_many_3d_points, 3) storing RGB colors for each 3D points from current
        iteration of plotting live
        """
        dimension = int(sqrt(how_many_3d_points))   # Calculating approximate size of new image with size more
        # compatible with 3D points

        resized_image = cv2.resize(rgb_image, (dimension, dimension))   # Creating new image with size more compatible
        # with 3D points

        colors_array = np.reshape(resized_image, (dimension * dimension,
                                                  3))   # Creating 2 dimensional vector from 3 dimensional image, so
        # the shape of this vector is more similar to shape of list with 3D points and consecutive RGB values of
        # pixels are corresponding to consecutive points in cloud of 3D points.

        colors_array = colors_array / 255   # Rescaling values of pixels from 0-255 to 0-1

        # Resizing vector with values of colors so the shape of it is exactly the same as shape of list with 3D points,
        # missing elements in new vector are fulfilled with copies of original vector
        colors_array = np.resize(colors_array, (how_many_3d_points, 3))

        return colors_array


if __name__ == "__main__":
    main()
