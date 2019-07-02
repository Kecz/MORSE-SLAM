import numpy as np
import cv2
from matplotlib import pyplot as plt


def main():
    # Creating class to detect landmarks
    detector = FindAndMatchLandmarks()

    # Creating orb detector
    detector.create_orb()

    # Aby wykryć landmarki ze zdjęcia:
    kp, des, img = detector.landmarks_from_path_to_image('obrazy/rgb0001.png')

    image_with_lm = detector.draw_markers_and_save("rgm_with_lm.png", img, kp)
    detector.draw_markers_and_show(img, kp)

    # Aby wykryć landmarki z pliku video:
    # detector.landmarks_from_movie('testowe_nagranie.webm')


class FindAndMatchLandmarks:
    """
    Class responsible for detecting landmarks on images.
    Before using its methods to detect landmarks, you must firstly create orb detector with method 'create_orb'.
    """
    def __init__(self):
        self.detector = None
        self.matcher = None

    def create_orb(self, maximum_amount_of_landmarks=2000, detector_edge_threshold=31, wta_param=2):
        """
        Method creating orb detector from Open-CV library (cv2).

        Orb has two parameters to adjust in this class:
        - nFeatures - maximum number of found KeyPoints.
        - edgeThreshold - parameter to tune detecting landmarks.

        :param maximum_amount_of_landmarks: maximum number of landmarks that will be detected. Default: 30
        :param detector_edge_threshold: Edge Threshold parameter used by orb detector to detect landmarks, for dark
        :param wta_param: The number of points that produce each element of the oriented BRIEF descriptor. Default 2.
        If you set this param to 3 or 4 remember that when using BFMatcher you need to use cv2.NORM_HAMMING2, otherwise
        use cv2.NORM_HAMMING
        images it is better to have lower value of this parameter. Default: 31

        """
        self.detector = cv2.ORB_create(nfeatures=maximum_amount_of_landmarks, edgeThreshold=detector_edge_threshold, WTA_K=wta_param)

    def create_akaze(self):
        """
        Method creating AKAZE detector from Open-CV library
        :return:
        """
        self.detector = cv2.AKAZE_create()

    def create_flann_for_orb(self, my_table_number=6, my_key_size=12, my_multi_probe_level=1, how_many_checks=100):
        FLANN_INDEX_LSH = 6         # Number of algorithm (6) that is very compatible with ORB - this algorithm is
                                    # based on searching decisive trees

        # params for ORB:
        index_params = dict(algorithm=FLANN_INDEX_LSH,
                            table_number=my_table_number,  # 6       # the number of hash tables to use, between 10 a 30 usually
                            key_size=my_key_size,          # 12      # the size of the hash key in bits (between 10 and 20 usually)
                            multi_probe_level=my_multi_probe_level)  # 1   # the number of bits to shift to check for neighboring buckets (0 is regular LSH, 2 is recommended).

        search_params = dict(checks=how_many_checks)  # or pass empty dictionary
        # The number of times the tree(s) in the index should be recursively traversed.
        # A higher value for this parameter would give better search precision, but also take more time.

        # create BFMatcher object
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

    def landmarks_from_path_to_image(self, path):
        """
        Method that loads image from given path and finds KeyPoints on it.

        :param path: path to png image on which landmarks are found
        :return: kp: list of KeyPoints, each Keypoint has many parameters such as pt - tuple location on image in pixels
                 des: descriptors of KeyPoints
                 img: image loaded from path with cv2
        """
        img = cv2.imread(path)

        # find the keypoints with ORB
        kp = self.detector.detect(img, None)

        # compute the descriptors with ORB
        kp, des = self.detector.compute(img, kp)

        return kp, des, img

    def landmarks_from_image(self, img):
        """
        Method that get images passed as numpy array and finds KeyPoints on it.

        :param img: image as numpy array, image can't have 4 dimensions, so you cannot pass RGBD image
        :return: kp: list of KeyPoints, each Keypoint has many parameters such as pt - tuple location on image in pixels
                 des: descriptors of KeyPoints
        """

        # find the keypoints with ORB
        kp = self.detector.detect(img, None)
        # kp = self.orb.detectAndCompute(img, None)

        # compute the descriptors with ORB
        kp, des = self.detector.compute(img, kp)

        return kp, des

    def draw_markers_and_save(self, new_path, img, kp):
        """
        Method that draws KeyPoints on image and saves new image at given path

        :param new_path: path where new image is saved
        :param img: image loaded with for example cv2
        :param kp: KeyPoints found on img

        :return: image with Keypoints drawn on it
        """

        # Adding markers to image
        image_with_lm = cv2.drawKeypoints(img, kp, outImage=np.array([]), color=(0, 255, 0),
                                          flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)

        # plt.imshow(img2), plt.show()
        plt.imsave(new_path, image_with_lm)

        return image_with_lm

    def draw_markers_and_show(self, img, kp):
        """
        Method that draws KeyPoints on image and shows it

        :param img: image loaded with for example cv2
        :param kp: KeyPoints found on img

        :return: image with Keypoints drawn on it
        """

        # Adding markers to image
        image_with_lm = cv2.drawKeypoints(img, kp, outImage=np.array([]), color=(0, 255, 0),
                                          flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)

        plt.imshow(image_with_lm), plt.show()

        return image_with_lm

    def landmarks_from_movie(self, source):
        """
        Method that plays given video with KeyPoints drawn on it in real time. To quit press q.

        :param source: video on which Keypoints are found
        """
        cap = cv2.VideoCapture(source)

        while True:
            ret, frame = cap.read()

            if frame is None:
                break

            kp = self.detector.detect(frame, None)

            kp, des = self.detector.compute(frame, kp)

            frame_with_lm = cv2.drawKeypoints(frame, kp, outImage=np.array([]), color=(0, 255, 0),
                                              flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)

            cv2.imshow('frame_with_lm', frame_with_lm)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
