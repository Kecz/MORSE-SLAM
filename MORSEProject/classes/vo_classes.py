import cv2
import numpy as np


class PinholeCamera:
    """
    Class representing Video Camera used by robot, it stores params of this camera needed in Visual Odometry
    """
    def __init__(self, width, height, int_matrix):
        self.cam_width = width
        self.cam_height = height
        self.fx = int_matrix[0][0]  # Focal of camera in x axis
        self.fy = int_matrix[1][1]  # Focal of camera in y axis
        self.cx = int_matrix[0][2]  # Principal point in x axis in pixels
        self.cy = int_matrix[1][2]  # Principal point in y axis in pixels
        self.intrinsic_matrix = int_matrix  # Intrinsic matrix of camera


class VisualOdometry:
    """
    Class which is responsible for full process of Visual Odometry.

    The whole process of using this class comes to use VisualOdometry.update(image) when passing consecutive frames from
    Video Camera, VisualOdometry will then count translation of robot, current coordinates of robot will be accessible
    at param "cur_coords" after passing second frame with update() because first frame is needed to initialize the
    whole process.
    """
    def __init__(self, cam):
        self.frame_stage = 0
        self.cam = cam
        self.cur_frame = None
        self.prev_frame = None
        self.cur_rotation = None
        self.prev_rotation = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        self.cur_translation = [[0], [0], [0]]
        self.prev_translation = [[0], [0], [0]]
        self.cur_coords = [[0], [0], [0]]
        self.base_coords = [[0], [0], [0]]
        self.kp_prev = None
        self.kp_cur = None
        self.des_prev = None
        self.des_cur = None

        self.detector = cv2.ORB_create(nfeatures=2000, edgeThreshold=31)
        # self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)

        # Choosing FlannBasedMatcher with params compatible with ORB detector as class matcher.
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH,
                            table_number=12,  # 6       # the number of hash tables to use, between 10 a 30 usually
                            key_size=20,  # 12          # the size of the hash key in bits (between 10 and 20 usually)
                            multi_probe_level=2)  # 1   # the number of bits to shift to check for neighboring buckets (0 is regular LSH, 2 is recommended).
        search_params = dict(checks=300)  # ilość sprawdzeń punktów im więcej tym dokładniej ale wolniej
        # self.kMinNumFeature = 500
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

    def get_position(self, scale):
        """

        :return:
        """

        x = self.cur_coords[0][0] / scale
        y = self.cur_coords[1][0] / scale
        z = self.cur_coords[2][0] / scale

        return [z, x, y]

    def processFirstFrame(self):
        """
        Processing first passed image to get from it KeyPoints and Descriptors which will be a reference for next image
        :return:
        """
        # self.kp_prev, self.des_prev = self.detector.detectAndCompute(self.cur_frame, None)
        self.kp_prev = self.detector.detect(self.cur_frame, None)
        self.kp_prev, self.des_prev = self.detector.compute(self.cur_frame, self.kp_prev)
        # self.kp_old = np.array([x.pt for x in self.kp_old], dtype=np.float32)
        self.frame_stage = 1
        self.prev_frame = self.cur_frame

    def processFrame(self):
        """
        Processing every next frame after the first one.
        Firstly new KeyPoints and Descriptors are obtained from new image. Then matches between previous and current
        descriptors are found. Then coordinates in pixels of each match are obtained. Then Essential Matrix is found
        using OpenCV function based on Ransac method. Then current position is obtained using OpenCV function
        'recoverPose'. With obtained values, new translation, rotation and coordinates are found. Current coordinates
        are accessible with 'cur_coords' param.
        :return:
        """
        # self.kp_cur, self.des_cur = self.detector.detectAndCompute(self.cur_frame, None)
        self.kp_cur = self.detector.detect(self.cur_frame, None)
        self.kp_cur, self.des_cur = self.detector.compute(self.cur_frame, self.kp_cur)

        try:

            matches = self.matcher.knnMatch(self.des_prev, self.des_cur, k=2)    #k - ile znajdzie matchy; jeśli 2 to szuka dwóch i jeśli znajdzie to wybiera ten który ma znacznie mniejszy dystans od drugiego; jeśli nie znalazł dwóch albo nie ma takiego z lepszym dystansem to nie znajduje żadnego

            matchesMask = [[0, 0] for i in range(len(matches))]
            good = []

            for i, match_lista in enumerate(matches):

                if len(match_lista) != 2:
                    continue
                m, n = match_lista[0], match_lista[1]

                if m.distance < 0.8 * n.distance:
                    matchesMask[i] = [1, 0]
                    good.append(m)
                # elif n.distance < 0.8 * m.distance:
                #     matchesMask[i] = [0, 1]
                #     good.append(n)
                # print(m)
                # print(n)
                # print('')

            kp1 = []
            kp2 = []
            for match in good:
                kp1.append(self.kp_prev[match.queryIdx].pt)
                kp2.append(self.kp_cur[match.trainIdx].pt)

            kp1 = np.asarray(kp1)
            kp2 = np.asarray(kp2)
            cam_matrix = np.asarray(self.cam.intrinsic_matrix)

            if len(kp1) > 5:

                E, mask = cv2.findEssentialMat(kp1, kp2, cam_matrix, prob=0.999, method=cv2.RANSAC)
                _, self.cur_rotation, self.cur_translation, mask = cv2.recoverPose(E, np.float64(kp1), np.float64(kp2), cam_matrix)

                self.cur_rotation = self.cur_rotation @ self.prev_rotation
                self.cur_translation = self.prev_translation + self.prev_rotation @ self.cur_translation
                self.cur_coords = self.cur_rotation @ self.base_coords + self.cur_translation

            self.kp_prev = self.kp_cur
            self.des_prev = self.des_cur
            self.prev_frame = self.cur_frame
            self.prev_rotation = self.cur_rotation
            self.prev_translation = self.cur_translation
        except:
            pass

    def update(self, img):
        """
        Appropriate processing of passed image
        :param img: input image, for example from Video Camera
        """
        self.cur_frame = img
        if self.frame_stage == 1:
            self.processFrame()
        elif self.frame_stage == 0:
            self.processFirstFrame()
        # self.last_frame = self.new_frame
