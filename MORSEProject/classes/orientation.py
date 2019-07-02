import numpy
from classes.quaternion import *
from classes.dataStreamReader import *
import threading
import math
import time


class Madgwick(threading.Thread):
    # IMU sensor needs to be rotated by PI/2 on x axis!

    # IMU data: timestamp, angular_velocity, linear_acceleration, magnetic_field
    IMU_FREQ = 60.0  # 60Hz
    GAIN = 0.56  # around 0.5 should be quite fast and without big overshoot

    def __init__(self, imu_data, sample_frequency=IMU_FREQ, gain=GAIN):
        """
        Initializes Madgwick's filter implementation.
        :param imu_data: need to receive object of type Reader that returns data from IMU
        :param sample_frequency: self explanatory - usually it is fixed frequency of simulation,
         didn't figure out how to change that tho
        :param gain: based on this settling time and overshoot will change
        """
        super(Madgwick, self).__init__()

        time.sleep(1)  # need to give some time to let data reader start or it will be a total failure
        self.imu_data = imu_data
        self.q = Quaternion()

        self.gain = gain
        self.sample_frequency = sample_frequency

        self.prev_data_time = time.time()
        self.prev_update_time = time.time()

        # self.roll = 0.0
        # self.pitch = 0.0
        # self.yaw = 0.0

    def roll(self):
        """
        Roll wrapper.
        :return: roll in degrees (-180, 180), base is 90 degrees, 180 when lying on right side,
         0 when lying on left side
        """
        roll, _, _ = self.q.quaternion_to_angles()

        return roll

    def pitch(self):
        """
        Pitch wrapper.
        :return: pitch in degrees (-90, 90), base is 0 degrees, positive values increase when looking down,
        negative values decrease when looking up
        """
        _, pitch, _ = self.q.quaternion_to_angles()

        return pitch

    def yaw(self):
        """
        Yaw wrapper.
        :return: yaw in degrees (-180, 180), base is 0 degrees, positive values when rotation left, negative values when
        rotating right, given that roll is positive, when reaches 180 the sign changes
        """
        _, _, yaw = self.q.quaternion_to_angles()

        return yaw

    def angles(self):
        """
        return all angles in degrees
        :return:
        """
        return self.q.quaternion_to_angles()

    def rate_of_change_of_quaternion_from_gyro(self, gyro):
        """
        Calculates rate of change of quaternion from gyroscope data.
        :param gyro: list [gx, gy, gz] with rad/s
        :return:
        """
        q_dot = Quaternion()
        q_dot.w = 0.5 * (-self.q.x * gyro[0] - self.q.y * gyro[1] - self.q.z * gyro[2])
        q_dot.x = 0.5 * (self.q.w * gyro[0] + self.q.y * gyro[2] - self.q.z * gyro[1])
        q_dot.y = 0.5 * (self.q.w * gyro[1] - self.q.x * gyro[2] + self.q.z * gyro[0])
        q_dot.z = 0.5 * (self.q.w * gyro[2] + self.q.x * gyro[1] - self.q.y * gyro[0])

        return q_dot

    def normalize_vec(self, vec):
        """
        Normalizes list of length 3
        :param vec: list [v1, v2, v3]
        :return: normalized list
        """
        # inv sqrt
        recip_norm = (vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2) ** -0.5
        vec[0] *= recip_norm
        vec[1] *= recip_norm
        vec[2] *= recip_norm

        return vec

    def update(self):
        """
        Heart of the algorithm. Collects imu data from data reader and iterates few times on one sample. Normalizes some
        things - actually You don't want to know what happens here. Just treat it like a big black box that gives
        orientation.
        Must be run as a thread!
        :return:
        """
        while True:
            # initial data sample
            acc = self.imu_data.get('linear_acceleration')
            gyro = self.imu_data.get('angular_velocity')
            mag = self.imu_data.get('magnetic_field')

            # data update every 16.66667ms
            if time.time() > self.prev_data_time + 1.0 / self.sample_frequency:
                # print("obtaining data")
                self.prev_data_time = time.time()
                acc = self.imu_data.get('linear_acceleration')
                gyro = self.imu_data.get('angular_velocity')
                mag = self.imu_data.get('magnetic_field')

            # madgwick update every 3.33333ms so it does update 5 times on each data sample
            if time.time() > self.prev_update_time + 1.0 / 5.0 / self.sample_frequency:
                # print("update")
                self.prev_update_time = time.time()

                q_dot = self.rate_of_change_of_quaternion_from_gyro(gyro)

                # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
                if not((acc[0] == 0.0) and (acc[1] == 0.0) and (acc[2] == 0.0)):
                    # Normalise accelerometer measurement
                    acc = self.normalize_vec(acc)

                    # Normalise magnetometer measurement
                    mag = self.normalize_vec(mag)

                    # Auxiliary variables to avoid repeated arithmetic
                    two_q0mx = 2.0 * self.q.w * mag[0]
                    two_q0my = 2.0 * self.q.w * mag[1]
                    two_q0mz = 2.0 * self.q.w * mag[2]
                    two_q1mx = 2.0 * self.q.x * mag[0]
                    two_q0 = 2.0 * self.q.w
                    two_q1 = 2.0 * self.q.x
                    two_q2 = 2.0 * self.q.y
                    two_q3 = 2.0 * self.q.z
                    two_q0q2 = 2.0 * self.q.w * self.q.y
                    two_q2q3 = 2.0 * self.q.y * self.q.z
                    q0q0 = self.q.w ** 2
                    q0q1 = self.q.w * self.q.x
                    q0q2 = self.q.w * self.q.y
                    q0q3 = self.q.w * self.q.z
                    q1q1 = self.q.x ** 2
                    q1q2 = self.q.x * self.q.y
                    q1q3 = self.q.x * self.q.z
                    q2q2 = self.q.y ** 2
                    q2q3 = self.q.y * self.q.z
                    q3q3 = self.q.z ** 2

                    # Reference direction of Earth's magnetic field
                    hx = mag[0] * q0q0 - two_q0my * self.q.z + two_q0mz * self.q.y + mag[0] * q1q1 \
                         + two_q1 * mag[1] * self.q.y + two_q1 * mag[2] * self.q.z - mag[0] * q2q2 - mag[0] * q3q3

                    hy = two_q0mx * self.q.z + mag[1] * q0q0 - two_q0mz * self.q.x + two_q1mx * self.q.y \
                         - mag[1] * q1q1 + mag[1] * q2q2 + two_q2 * mag[2] * self.q.z - mag[1] * q3q3

                    two_bx = math.sqrt(hx * hx + hy * hy)

                    two_bz = -two_q0mx * self.q.y + two_q0my * self.q.x + mag[2] * q0q0 + two_q1mx * self.q.z \
                             - mag[2] * q1q1 + two_q2 * mag[1] * self.q.z - mag[2] * q2q2 + mag[2] * q3q3

                    four_bx = 2.0 * two_bx
                    four_bz = 2.0 * two_bz

                    # Gradient decent algorithm corrective step
                    s = Quaternion()

                    s.w = -two_q2 * (2.0 * q1q3 - two_q0q2 - acc[0]) + two_q1 * (2.0 * q0q1 + two_q2q3 - acc[1])
                    - two_bz * self.q.y * (two_bx * (0.5 - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mag[0])
                    + (-two_bx * self.q.z + two_bz * self.q.x) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - mag[1])
                    + two_bx * self.q.y * (two_bx * (q0q2 + q1q3) + two_bz * (0.5 - q1q1 - q2q2) - mag[2])

                    s.x = two_q3 * (2.0 * q1q3 - two_q0q2 - acc[0]) + two_q0 * (2.0 * q0q1 + two_q2q3 - acc[1]) \
                         - 4.0 * self.q.x * (1 - 2.0 * q1q1 - 2.0 * q2q2 - acc[2]) + two_bz * self.q.z * (two_bx * (0.5 - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mag[0]) \
                         + (two_bx * self.q.y + two_bz * self.q.w) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - mag[1]) \
                         + (two_bx * self.q.z - four_bz * self.q.x) * (two_bx * (q0q2 + q1q3) + two_bz * (0.5 - q1q1 - q2q2) - mag[2])

                    s.y = -two_q0 * (2.0 * q1q3 - two_q0q2 - acc[0]) + two_q3 * (2.0 * q0q1 + two_q2q3 - acc[1]) \
                         - 4.0 * self.q.y * (1 - 2.0 * q1q1 - 2.0 * q2q2 - acc[2]) + (-four_bx * self.q.y - two_bz * self.q.w) \
                         * (two_bx * (0.5 - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mag[0]) + (two_bx * self.q.x + two_bz * self.q.z) \
                         * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - mag[1]) + (two_bx * self.q.w - four_bz * self.q.y) \
                         * (two_bx * (q0q2 + q1q3) + two_bz * (0.5 - q1q1 - q2q2) - mag[2])

                    s.z = two_q1 * (2.0 * q1q3 - two_q0q2 - acc[0]) + two_q2 * (2.0 * q0q1 + two_q2q3 - acc[1]) \
                         + (-four_bx * self.q.z + two_bz * self.q.x) * (two_bx * (0.5 - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mag[0]) \
                         + (-two_bx * self.q.w + two_bz * self.q.y) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - mag[1]) \
                         + two_bx * self.q.x * (two_bx * (q0q2 + q1q3) + two_bz * (0.5 - q1q1 - q2q2) - mag[2])

                    # Normalise
                    s.normalize()

                    # Apply feedback step
                    q_dot.w -= self.gain * s.w
                    q_dot.x -= self.gain * s.x
                    q_dot.y -= self.gain * s.y
                    q_dot.z -= self.gain * s.z

                # Integrate rate of change of quaternion to yield quaternion
                self.q.w += q_dot.w * (1.0 / self.sample_frequency)
                self.q.x += q_dot.x * (1.0 / self.sample_frequency)
                self.q.y += q_dot.y * (1.0 / self.sample_frequency)
                self.q.z += q_dot.z * (1.0 / self.sample_frequency)

                # // Normalise quaternion
                self.q.normalize()

    def run(self):
        """
        Runs the update method as a thread.
        :return:
        """
        update_thread = threading.Thread(target=self.update)
        update_thread.start()


#
# def main():
#     # create reader to obtain live data from IMU
#     imu_data = Reader('localhost', 60009)
#     imu_data.start()
#
#     # get Madgwick filter running to calc orientation
#     orientation = Madgwick(imu_data)
#     orientation.start()
#
#     while True:
#         time.sleep(0.5)
#         print(orientation.roll(), orientation.pitch(), orientation.yaw())
#         # print(orientation.q.w, orientation.q.x, orientation.q.y, orientation.q.z)
#         pass
#
# if __name__=='__main__':
#     main()


