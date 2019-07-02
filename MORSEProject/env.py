from morse.builder import *
import math
# =====================================================================================
# Sensors frequencies
IMU_FREQUENCY = 30  # 60Hz
LIDAR_FREQUENCY = 30
POSE_FREQUENCY = 30
DCAM_FREQUENCY = 30
VIDEOCAMERA_FREQUENCY = 30
ODO_FREQUENCY = 30

# Sockets ports
videocamera_port = 60011
pose_port = 60007
depthcamera_port = 60012
lidar_port = 60008
imu_port = 60009
laser_port = 60010
right_laser_port = 60013
left_laser_port = 60014
odometry_port = 60006

# Initial position of sensors
TRANS_X = 0.5
TRANS_Y = 0.0
TRANS_Z = 0.3
# =====================================================================================
# Adding robot
atrv = ATRV()
# =====================================================================================
# Adding motion controller
motion = MotionVW()

motion.translate(z=0.3)
motion.add_service('socket')
motion.add_stream('socket')

atrv.append(motion)
# =====================================================================================
# Adding built-in keyboard control
keyboard = Keyboard()
keyboard.properties(ControlType='Position')

atrv.append(keyboard)
# =====================================================================================
# Adding video camera sensor
videocamera = VideoCamera()
videocamera.translate(TRANS_X, TRANS_Y, TRANS_Z)
# For our 2D map videocamera can't be rotated along z axis because if it is rotated, the images captured from it, will
# be rotated also

# Both cameras are rotated a bit along y axis so they are looking a bit more higher and they are seeing less floor
# and this is an advantage because floor disturbs finding KeyPoints
videocamera.rotate(0, math.pi/8, 0)    # te math.pi/10 jest to lekkie przekręcenia kamery, żeby patrzyła bardziej w sufit, niż w podłogę
videocamera.properties(cam_width=100)
videocamera.properties(cam_height=100)
videocamera.properties(cam_far=8)       # Jesli ustawi się maksymalny zasięg to kamera w miejsca których nie widzi wstawi kolor szary
videocamera.properties(cam_focal=25)
videocamera.properties(retrieve_depth=False)
videocamera.properties(capturing=True)
videocamera.add_stream('socket', port=videocamera_port)
videocamera.add_service('socket')
videocamera.frequency(VIDEOCAMERA_FREQUENCY)

atrv.append(videocamera)
# =====================================================================================
# Adding Pose sensor
pose = Pose()

pose.translate(TRANS_X, TRANS_Y, TRANS_Z)
pose.frequency(POSE_FREQUENCY)
pose.add_stream('socket', port=pose_port)
pose.add_service('socket')

atrv.append(pose)
# ======================================================================================
# videocamera.profile()     # Displaying parameters of camera in real time in simulation

dcam = DepthCamera()
dcam.properties(cam_width=15)
dcam.properties(cam_height=15)
dcam.properties(cam_far=8.0)
dcam.frequency(DCAM_FREQUENCY)
dcam.translate(TRANS_X, TRANS_Y, TRANS_Z)       # przesuniecie kamery do przodu robota o 0.5 i do góry o 0.5
dcam.rotate(0.0, -math.pi/8, math.pi)       # te -math.pi/8 w osi y służy temu aby dcam patrzyło bardziej do góry, czyli widziało mniej podłogi, a ten math.pi w osi z jest ponieważ DepthCamera domyślnie jest do góry nogami nie wiadomo czemu
dcam.add_stream('socket', port=depthcamera_port)
dcam.add_interface('socket')
atrv.append(dcam)

# ======================================================================================
# Second VideoCamera which is displaying current view in simulation. For our 2D map we can't use the previous
# VideoCamera because video displayed in simulation is a mirror reflection of true view, so we have to rotate camera
# along z-axis by 180 deg and set Vertical_Flip=False to have real video.

# For built-in environment named 'indoors-1/indoor-1' you also have to set Vertical_Flip to False and rotate camera
# along z-axis by math.pi to get real view from camera, not mirror image.

camera_to_display = VideoCamera()
camera_to_display.frequency(25)
# Camera is translated so it is located somewhere at the front of the roof
camera_to_display.translate(TRANS_X, TRANS_Y, TRANS_Z)
camera_to_display.rotate(0, -math.pi/10, math.pi)
camera_to_display.properties(Vertical_Flip=False)
atrv.append(camera_to_display)

# ======================================================================================
# Adding Lidar sensor
lidar = Sick()

lidar.properties(Visible_arc=False)  # arc is invisible during the simulation
lidar.properties(laser_range=8.0)   # range - distance to objects [meters]
lidar.properties(resolution=5)      # angle between each laser in the sensor [degrees]
lidar.properties(scan_window=180)
lidar.translate(TRANS_X, TRANS_Y, TRANS_Z)
lidar.frequency(LIDAR_FREQUENCY)    # the refresh frequency of the sensor
lidar.add_stream('socket', port=lidar_port)
lidar.add_service('socket')

atrv.append(lidar)

# ======================================================================================
# Adding IMU
imu = IMU()
# need to rotate sensor in roll axis by 90 degrees, don't ask why
imu.rotate(3.14/2, 0, 0)
# set frequency of IMU
imu.frequency(IMU_FREQUENCY)
# add some noise to the sensors - this doesn't affect magnetometer data
imu.alter('IMUNoise', gyro_std=0.05, accel_std=0.05)  # standard deviations

imu.add_stream('socket', port=imu_port)
imu.add_service('socket')

atrv.append(imu)
# ======================================================================================
# Adding long laser to measure distance in front of the vehicle
laser = Sick()
laser.translate(0, 0, TRANS_Z)
laser.properties(Visible_arc=False)  # arc is invisible during the simulation
laser.properties(laser_range=2.0)  # range - distance to objects [meters]
laser.properties(resolution=5)  # angle between each laser in the sensor [degrees]
laser.properties(scan_window=20)
laser.frequency(LIDAR_FREQUENCY)  # the refresh frequency of the sensor
laser.add_stream('socket', port=laser_port)
laser.add_service('socket')

atrv.append(laser)

# ======================================================================================
# Adding long laser to measure distance on the right side of the vehicle
right = Sick()
right.translate(0, 0, TRANS_Z)
right.rotate(0, 0, -math.pi/2)
right.properties(Visible_arc=False)  # arc is invisible during the simulation
right.properties(laser_range=2.0)  # range - distance to objects [meters]
right.properties(resolution=5)  # angle between each laser in the sensor [degrees]
right.properties(scan_window=20)
right.frequency(LIDAR_FREQUENCY)  # the refresh frequency of the sensor
right.add_stream('socket', port=right_laser_port)
right.add_service('socket')

atrv.append(right)

# ======================================================================================
# Adding long laser to measure distance on the left side of the vehicle
left = Sick()
left.translate(0, 0, TRANS_Z)
left.rotate(0, 0, math.pi/2)
left.properties(Visible_arc=False)  # arc is invisible during the simulation
left.properties(laser_range=2.0)  # range - distance to objects [meters]
left.properties(resolution=5)  # angle between each laser in the sensor [degrees]
left.properties(scan_window=20)
left.frequency(LIDAR_FREQUENCY)  # the refresh frequency of the sensor
left.add_stream('socket', port=left_laser_port)
left.add_service('socket')

atrv.append(left)

# ======================================================================================
# Adding encoder
odometry_differential=Odometry()
odometry_differential.level('differential')
odometry_differential.frequency(ODO_FREQUENCY)
odometry_differential.add_stream('socket', port=odometry_port)
odometry_differential.add_service('socket')
atrv.append(odometry_differential)
# ======================================================================================
# Remember that environment must be appended at the end of script!

env = Environment('env_3D/Rampa_duza_bez_sufitu2.blend', fastmode=False)
env.set_camera_location([-10, 20, 35])
env.set_camera_rotation([0.4, 0, math.pi])
env.select_display_camera(camera_to_display)
# needed data for magnetometer to work, set for ETI
env.properties(longitude=54.371209, latitude=18.613334, altitude=30.0)