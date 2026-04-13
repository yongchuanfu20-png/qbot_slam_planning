#-----------------------------------------------------------------------------#
#------------------Skills Progression 1 - Task Automation---------------------#
#-----------------------------------------------------------------------------#
#-------------------------Lab 4 - Obstacle Detection--------------------------#
#-----------------------------------------------------------------------------#

# Imports
from pal.products.qbot_platform import QBotPlatformDriver,\
    QBotPlatformCSICamera, QBotPlatformRealSense, QBotPlatformLidar
from hal.content.qbot_platform_functions import QBPVision, QBPRanging
from quanser.hardware import HILError
from pal.utilities.probe import Probe
from pal.utilities.gamepad import LogitechF710
import time
import numpy as np
import cv2
from pal.utilities.math import Calculus
import os
import AstarPathPlanner as ap
import Controller as cont
import Asearch as aas


# Section A - Setup
os.system('quarc_run -q -Q -t tcpip://localhost:17000 *.rt-linux_qbot_platform -d /tmp')
time.sleep(5)
os.system('quarc_run -r -t tcpip://localhost:17000 qbot_platform_driver_physical.rt-linux_qbot_platform  -d /tmp -uri tcpip://localhost:17099')
time.sleep(3)
print('driver loaded')

ipHost, ipDriver = '192.168.1.19', 'localhost'
commands, arm, noKill = np.zeros((2), dtype = np.float64), 0, True
frameRate, sampleRate = 60.0, 1/60.0
counter, counterDown, counterLidar = 0, 0, 0
endFlag, obstacle, offset, forSpd, turnSpd = False, False, 0, 0, 0
startTime = time.time()
def elapsed_time():
    return time.time() - startTime
timeHIL, prevTimeHIL = elapsed_time(), elapsed_time() - 0.017

LIDAR_POS_X = QBotPlatformDriver.LIDAR_POS_X
LIDAR_POS_Y = QBotPlatformDriver.LIDAR_POS_Y


try:
    # Section B - Initialization
    myQBot       = QBotPlatformDriver(mode=1, ip=ipDriver)
    downCam      = QBotPlatformCSICamera(frameRate=frameRate, exposure = 39.0, gain=17.0)
    lidar        = QBotPlatformLidar()
    gpad         = LogitechF710(1)
    vision       = QBPVision()
    ranging      = QBPRanging()
    probe        = Probe(ip = ipHost)
    probe.add_display(imageSize = [800, 800, 1], scaling = True, scalingFactor= 2, name='Raw Image')
    probe.add_display(imageSize=[800, 800, 1], scaling=True, scalingFactor=2, name='Path')
    # probe.add_plot(numMeasurements=1680, scaling=True, scalingFactor=4, name='Raw Lidar')
    # probe.add_plot(numMeasurements=410, scaling=False, name='Plotting Lidar')
    line2SpdMap = vision.line_to_speed_map(sampleRate=sampleRate, saturation=75)
    next(line2SpdMap)
    startTime = time.time()
    time.sleep(0.5)

    pose = (0, 0, 0)
    timelimit = 0
    world_frame = np.zeros((800, 800), np.uint8)

    world_frame_op = np.zeros((800, 800), np.uint8)

    path_map = np.zeros((800, 800), np.uint8)

    prev_time = time.perf_counter()
    prev_wp = None
    timelimit = 0
    nodelimit = 0

    path_generated = False
    map_generated = False

    # Main loop
    while noKill and not endFlag:
        t = elapsed_time()

        if not probe.connected:
            probe.check_connection()

        if probe.connected:

            # Joystick Driver
            newGPad = gpad.read()
            arm = gpad.buttonLeft and not obstacle
            lineFollow = gpad.buttonA
            if obstacle:
                arm = 0
            turnStick = gpad.leftJoystickX
            driveStick = gpad.rightJoystickY
            if gpad.buttonRight:
                noKill = False

            # Section C - toggle line following
            if not lineFollow:
                forSpd=0.7*driveStick/2
                turnSpd=3.564*turnStick/2
                commands = np.array([forSpd, turnSpd], dtype = np.float64) # robot spd command
            else:
                if path_generated == False:
                    path_generated = True
                    res = 0.025
                    c_space = aas.compute_c_space(world_frame, 2)
                    col_w = int(-(pose[0] / res) + 400)
                    row_w = int(-(pose[1] / res) + 400)
                    path, vv, nn = aas.search(c_space, (col_w, row_w), (400, 450), "a_star", "euclidean")
                    path_map = c_space.copy()
                    for n in path:
                        path_map[n] = 255

                    config = cont.PurePursuitConfig()
                    controller = cont.PurePursuitController(path, config)

                state = cont.RobotState(x=pose[0], y=pose[1], theta=pose[2])
                v_cmd, omega_cmd, wl, wr, lookahead = controller.compute_control(state)

                forSpd, turnSpd = v_cmd, omega_cmd
                if controller.is_goal_reached(state):
                    forSpd, turnSpd = 0, 0
                    print('Goal Reached')
                    path_generated = False
                print('speed:', forSpd, 'turn:', turnSpd)
                commands = np.array([forSpd, turnSpd], dtype=np.float64)  # robot spd command

            # QBot Hardware
            newHIL = myQBot.read_write_std(timestamp = time.time() - startTime,
                                            arm = arm,
                                            commands = commands, userLED=obstacle)
            if newHIL:
                timeHIL     = elapsed_time()
                newDownCam  = downCam.read()
                newLidar    = lidar.read()

                if newDownCam:
                    counterDown += 1

                    # Section D - Image processing 

                    # Section D.1 - Undistort and resize the image
                    undistorted = vision.df_camera_undistort(downCam.imageData)
                    gray_sm = cv2.resize(undistorted, (320, 200))

                    #-------Replace the following line with your code---------#
                    # Subselect a part of the image and perform thresholding
                    binary = None

                    # Blob Detection via Connected Component Labeling
                    col, row, area = 0, 0, 0

                    # Section D.2 - Speed command from blob information
                    forSpd, turnSpd = line2SpdMap.send((col, 1, 0))
                    #---------------------------------------------------------#
                
                if newLidar:
                    counterLidar += 1

                    # Section E - LiDAR processing 
                    #-------Replace the following line with your code---------#
                    rangesAdj, anglesAdj = ranging.adjust_and_subsample(lidar.distances, lidar.angles, 1260, 3)
                    rangesC, anglesC = ranging.correct_lidar([myQBot.LIDAR_POS_X, myQBot.LIDAR_POS_Y], rangesAdj,
                                                             anglesAdj)

                    # ---------------------------------------------------------#

                    # Section F - Obstacles detection
                    # -------lidar map grid generation---------#
                    minThreshold = 0.2858
                    rangesP, anglesP, obstacle = ranging.detect_obstacle(rangesC, anglesC, forSpd, 0.5, turnSpd, 1,
                                                                         minThreshold, 10)
                    lidar_grid = ap.lidarmapgrid(rangesC, anglesC)
                    #---------------------------------------------------------#

                    # ----------------------------SLAM  1: GET POSE   put under if newlidar?? --------------------------#

                    timelimit += 1

                    if timelimit % 5 == 1:

                        wp = myQBot.wheelPositions  # [L, R] rad
                        gyro = myQBot.gyroscope  # [gx, gy, gz] rad/s

                        now = time.perf_counter()
                        dt = now - prev_time
                        prev_time = now

                        # skip pose update on first iteration only
                        if prev_wp is None:
                            prev_wp = wp.copy()
                        else:
                            pose, prev_wp = ap.update_pose_diffdrive(
                                pose=pose,
                                wheel_positions=wp,
                                prev_wheel_positions=prev_wp,
                                dt=dt,
                                wheel_radius=myQBot.WHEEL_RADIUS,
                                wheel_base=myQBot.WHEEL_BASE,
                                gyroscope=gyro,
                                gyro_weight=0,
                            )

                        world_frame = ap.place_local_into_global(lidar_grid, pose, world_frame)

                        # ------------------------------------IM SHOW -----------------------------------
                        world_frame_location = world_frame.copy()
                        r, c = pose[0], pose[1]
                        r = -r / 0.025 + 400
                        c = -c / 0.025 + 400
                        robot_radius = 10
                        for dr in range(-robot_radius, robot_radius + 1):
                            for dc in range(-robot_radius, robot_radius + 1):
                                # Check Euclidean distance for circular robot [cite: 27, 29]
                                if np.sqrt(dr ** 2 + dc ** 2) <= robot_radius:
                                    nr, nc = r + dr, c + dc
                                    world_frame_location[int(nr), int(nc)] = 255



                if counterDown%4 == 0:
                    sending = probe.send(name='Raw Image', imageData= world_frame_location)
                # if counterLidar % 2 == 0:
                #     sending = probe.send(name='Raw Lidar', lidarData=(rangesC, anglesC))
                if counterLidar % 2 == 1:
                    sending = probe.send(name='Path', imageData= path_map)

                prevTimeHIL = timeHIL

except KeyboardInterrupt:
    print('User interrupted.')
except HILError as h:
    print(h.get_error_message())
finally:
    lidar.terminate()
    downCam.terminate()
    myQBot.terminate()
    probe.terminate()
    os.system('quarc_run -q -Q -t tcpip://localhost:17000 *.rt-linux_qbot_platform -d /tmp')