from classes.dcam_odo_lidar2 import DepthCamera
from classes.dcam_pose_lidar2 import DepthCameraPose
from classes.kasiek_control import Control
from classes.odometry import Odometry
from classes.video_camera_server import VideoCameraServer
from classes.images_processing_class import ImagesProcessing
import time


class MegaMain:
    FREQUENCY = 30
    HOW_LONG = 1000
    SCALE = 1.2  # for visual odometry - dependent on speed of simulaiton

    def __init__(self, ax2D, ax3D, ax2Dpose, ax3Dpose, how_long=HOW_LONG, frequency=FREQUENCY, scale=SCALE):
        # duration of simulation
        self.how_long = how_long
        # frequency of sensors and other stuff
        self.frequency = frequency
        # scale for camera trajectory - needs to be found out experimentally
        self.scale = scale

        # autonomous control of the robot
        self.control = Control()
        # imu data reader for obtaining readings from this sensor
        # self.imu_data = Reader('localhmegaost', 60009)
        # self.imu_data.start()self.simtime,

        # madgwick algorithm implementation for calculating orientation of the robot
        # self.madgwick = Madgwick(self.imu_data, sample_frequency=self.frequency)
        # self.madgwick.start()  # starting thread so the calculations are in background

        # instnce allowing to calculate trajectory from imu and laser
        # self.imu_lidar_pose = ObservedPose()

        # instnce allowing to calculate trajectory from odometry
        self.odometry_pose = Odometry()

        # create instance responsible for mapping
        self.dcam_mapping = DepthCamera(ax2D, ax3D)
        self.dcam_pose = DepthCameraPose(ax2Dpose, ax3Dpose)
        # self.dcam_mapping.start()  # start mapping in background
        # FILTERS
        # self.complementary_filter = ComplementaryFilter()
        # create instance of kalman filter that will help estimating current position
        # self.kalman = Kalman3D(dt=1 / self.frequency)

        # VIDEO CAMERA STUFF
        self.video_camera_server = VideoCameraServer(host='localhost', port=60011)
        self.video_camera_server.run()
        # camera_params = self.video_camera_server.get_all()['intrinsic_matrix']

        # cam = PinholeCamera(image_width, image_height, camera_params)
        # self.vo = VisualOdometry(cam)

        self.image_processing = ImagesProcessing()

        # give some time to initialize sensors, streams and whatever
        time.sleep(1)
        self.image_height = self.video_camera_server.current_data['height']
        self.image_width = self.video_camera_server.current_data['width']

        # self.sim_thread = threading.Thread(target=self.run_simulation)
        # self.sim_thread.start()

    def run_simulation(self):
        """

        :return:
        """
        prev_time = time.time()

        time_end = time.time() + self.how_long

        actions_count = 0  # how many actions to take between mapping

        while time.time() < time_end:
            # make it run on specified speed; still could sub the time of algorithm execution tho
            if time.time() >= prev_time + 1 / self.frequency:
                # print(time.time() - prev_time)
                prev_time = time.time()

                # update position from imu_lidar
                # self.imu_lidar_pose.update(self.madgwick.angles(), control.rotate)
                # self.imu_lidar_pose.update(self.madgwick.angles(), False)

                # acquire accelerations
                # acc = self.imu_data.get('linear_acceleration')
                # sensor is tilted on the side
                # acc = [acc[0], -acc[2], acc[1]]

                # imu_lidar actual position
                # il_pos = self.imu_lidar_pose.get_observed_position()  # [x, y, z]

                # odometry actual position
                odo_x, odo_y, odo_z, _, _, _ = self.odometry_pose.get_data()  # [x, y, z]
                odo_pos = [odo_x, odo_y, odo_z]

                # Geting image from VideoCamera in binary format
                binary_image = self.video_camera_server.get_image()

                # Creating rgb image from data captured from videocamera
                rgb_image = self.image_processing.create_rgb_image(binary_image, self.image_height, self.image_width)

                # self.vo.update(rgb_image)
                # vo_pos = self.vo.get_position(SCALE)

                # filtered_pos = self.complementary_filter.fusion(odo_pos, odo_pos, odo_pos)

                # kalman estimate current position
                # self.kalman.update(filtered_pos, acc)
                # kalman_pos = self.kalman.get_estimated_position()

                if actions_count >= 100:
                    actions_count = 0

                    # stop robot before plotting
                    self.control.set_motion(self.control.stop)

                    # update position for mapping
                    self.dcam_mapping.set_position(odo_pos)
                    self.dcam_pose.set_position(odo_pos)
                    # map the terrain
                    self.dcam_mapping.points_cloud_update(rgb_image)
                    self.dcam_pose.points_cloud_update(rgb_image)

                    # start robot after plotting(the move it was doing)
                    self.control.set_motion(self.control.last_v_w)

                # make a move based on current position
                self.control.update(odo_pos)

                actions_count += 1


if __name__ == "__main__":
    megamain = MegaMain()
    megamain.run_simulation()