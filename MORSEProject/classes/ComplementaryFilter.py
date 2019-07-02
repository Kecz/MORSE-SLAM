

class ComplementaryFilter():
    def __init__(self, imu_lidar_coefficient=0.02, odometry_coefficient=0.96, vo_coefficient=0.02):
        """
        Filter for trajectory data FUSION
        :param imu_lidar_coefficient:  importance of data from this source
        :param dcam_lidar_coefficient: importance of data from this source
        :param vo_coefficient: importance of data from this source
        """
        self.il_co = imu_lidar_coefficient
        self.odo_co = odometry_coefficient
        self.vo_co = vo_coefficient

        self.normalise_coefficionets()

    def normalise_coefficionets(self):
        """
        Normalise coefficients if the ones given(not default) does not sum up to 1
        :return:
        """
        sum = self.il_co + self.odo_co + self.vo_co
        self.il_co /= sum
        self.odo_co /= sum
        self.vo_co /= sum

    def fusion(self, il_data, odo_data, vo_data):
        """
        Receive position from all the sensors we have and then calculate weighted average
        :param il_data:
        :param odo_data:
        :param vo_data:
        :return: fusioned data
        """
        x = self.il_co * il_data[0] + self.odo_co * odo_data[0] + self.vo_co * vo_data[0]
        y = self.il_co * il_data[1] + self.odo_co * odo_data[1] + self.vo_co * vo_data[1]
        z = self.il_co * il_data[2] + self.odo_co * odo_data[2] + self.vo_co * vo_data[2]
        return [x, y, z]
