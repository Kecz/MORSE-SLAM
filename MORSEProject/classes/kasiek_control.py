from classes.dataStreamReader import *
import socket


class Control:
    """
    Class which control movement of robot - it can only move forward or can only rotate at the same time
    """
    def __init__(self, host='localhost', port=4000):
        """
        Initialize class and parameters of beams and maximum range to obstacles
        detect_front(bool): flag that is true where obstacle was detected in front of the robot
        detect_left(bool): flag that is true where obstacle was detected on the left side of the robot
        detect_right(bool): flag that is true where obstacle was detected on the right side of the robot
        detect_left_closer(bool): flag that is true where obstacle was detected left side of the robot closer than 2.5 meters
        detect_front_closer(bool): flag that is true where obstacle was detected in front of the robot closer than 0.8 meters
        """
        self.front_beams = 0
        self.right_beams = 0
        self.left_beams = 0
        # flags if obstacle is detected
        self.detect_front = False
        self.detect_left = False
        self.detect_right = False
        self.visited_x = []
        self.visited_y = []
        # since actions are independent they can be grouped like this for more clarity in code
        self.move_forward = {"v": 1, "w": 0}
        self.move_slower = {"v": 0.7, "w": 0}
        self.move_back = {"v": -1, "w": 0}
        self.turn_left = {"v": 0, "w": -0.5}
        self.turn_right = {"v": 0, "w": -0.5}
        # self.after = {"v": -0.5, "w": -0.5}
        self.stop = {"v": 0, "w": 0}
        self.start = False
        self.bylem_x = 100000
        self.bylem_y = 100000
        self.bylem = 0
        self.laser = Reader('localhost', 60010)
        self.right = Reader('localhost', 60013)
        self.left = Reader('localhost', 60014)
        self.laser.run()
        self.right.run()
        self.left.run()
        self.no_move = False

        # for communiction with motion actuator
        self.host = host
        self.port = port
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            self.sock.connect((host, port))
        except:
            print("socket connection failed")

        v_w = self.stop  # initial movement

        # last movement will be compared to next movement, if they are different robot will be forced to stop
        # between movements
        self.last_v_w = v_w

        self.set_motion(v_w)

    def update(self, data_pose):
        """
        Function that control movement of the robot and adding visited points to list.
        v - linear speed
        w - angular speed
        :param data_pose: position obtained from some source in main loop [x, y, z]
        """
        if not self.start:
            v_w = self.move_forward
            self.start = True
            self.last_v_w = v_w
        # checking that is object in range of lasers sensor beams
        self.reset_collision()
        self.object_detection(self.laser.get_all(), self.right.get_all(), self.left.get_all())
        # checking that point is in list of visited points
        if round(data_pose[0], 3) in self.visited_x and round(data_pose[1], 3) in self.visited_y:
            if round(data_pose[0], 3) == self.bylem_x and round(data_pose[1], 3) == self.bylem_y:
                self.bylem += 1
                if self.bylem < 3:
                    v_w = {"v": 0.5, "w": -0.5}
                else:
                    v_w = {"v": -2, "w": -1}
                self.last_v_w = v_w
            elif round(data_pose[0], 3) != self.bylem_x and round(data_pose[1], 3) != self.bylem_y:
                v_w = {"v": 0.5, "w": -0.5}
                self.bylem_x = round(data_pose[0], 3)
                self.bylem_y = round(data_pose[1], 3)
                self.bylem = 0
                self.last_v_w = v_w

        else:
            self.visited_x.append(round((data_pose[0]), 3))
            self.visited_y.append(round((data_pose[1]), 3))
            # according of detected obstacles set v (speed) and w (rotate) of the robot
            if self.detect_front and self.detect_left and self.detect_right:
                v_w = self.turn_left
            elif self.detect_left and self.detect_front:
                v_w = self.turn_right
            elif self.detect_right and self.detect_front:
                v_w = self.turn_left
            elif self.detect_front:
                v_w = self.turn_right
            else:
                v_w = self.move_forward
            self.last_v_w = v_w

        try:
            self.set_motion(v_w)
        except:
            self.set_motion(self.last_v_w)

    def reset_collision(self):
        self.front_beams = 0
        self.right_beams = 0
        self.left_beams = 0
        self.detect_front = False
        self.detect_left = False
        self.detect_right = False

    def object_detection(self, laser_data, right_data, left_data):
        """
            Function that checks ability to ride. It returns flags for directions where obstacles were detected.
        """
        range_laser = laser_data.get("range_list")
        range_left = left_data.get("range_list")
        range_right = right_data.get("range_list")

        for i in range(len(range_laser)):
            if range_laser[i] < 1.5:
                self.front_beams += 1
        for i in range(len(range_left)):
            if range_left[i] < 1.5:
                self.left_beams += 1
        for i in range(len(range_right)):
            if range_right[i] < 1.5:
                self.right_beams += 1
        if self.front_beams > 0:
            self.detect_front = True
        else:
            self.detect_front = False
        if self.left_beams > 0:
            self.detect_left = True
        else:
            self.detect_left = False
        if self.right_beams > 0:
            self.detect_right = True
        else:
            self.detect_right = False

    def set_motion(self, v_w):
        """
        sends given speed and angular speed to the motion actuator through connected socket
        :param v_w: dictionary consisting of v: linear speed (-1, 1) back/front, w: angular speed (-0.5, 0.5) left/right
        :return:
        """
        try:
            msg = "id1 atrv.motion set_speed [%f, %f]\n" % (v_w["v"], v_w["w"])
            self.sock.send(msg.encode("utf-8"))
        except Exception as error_msg:
            print(error_msg)