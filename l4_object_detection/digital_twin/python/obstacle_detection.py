#-----------------------------------------------------------------------------#
#------------------Skills Progression 1 - Task Automation---------------------#
#-----------------------------------------------------------------------------#
#-------------------------Lab 4 - Obstacle Detection--------------------------#
#-----------------------------------------------------------------------------#
from asyncio.windows_events import NULL
from webbrowser import Edge

from numpy.f2py.crackfortran import word_pattern
# Imports
from pal.products.qbot_platform import QBotPlatformDriver,Keyboard,\
    QBotPlatformCSICamera, QBotPlatformRealSense, QBotPlatformLidar
from hal.content.qbot_platform_functions import QBPVision, QBPRanging
from quanser.hardware import HILError
from pal.utilities.probe import Probe
from pal.utilities.gamepad import LogitechF710
import time
import numpy as np
import cv2
from pal.utilities.math import Calculus
from qlabs_setup import setup
import AstarPathPlanner as ap
import Controller as cont
import Asearch as aas
import dstar as ds

# Section A - Setup

setup(locationQBotP=[-1.35, 0.3, 0.05], rotationQBotP=[0, 0, 0], verbose=True)
time.sleep(2)
ipHost, ipDriver = 'localhost', 'localhost'
commands, arm, noKill = np.zeros((2), dtype = np.float64), 0, True
frameRate, sampleRate = 60.0, 1/60.0
counter, counterDown, counterLidar = 0, 0, 0
endFlag, obstacle, offset, forSpd, turnSpd = False, False, 0, 0, 0
startTime = time.time()
def elapsed_time():
    return time.time() - startTime
timeHIL, prevTimeHIL = elapsed_time(), elapsed_time() - 0.017

# cv2.namedWindow("LiDAR", cv2.WINDOW_NORMAL)
# cv2.namedWindow("World_Frame", cv2.WINDOW_NORMAL)
# cv2.namedWindow("path", cv2.WINDOW_NORMAL)
# cv2.namedWindow("Optimise_Frame", cv2.WINDOW_NORMAL)


try:
    # Section B - Initialization
    myQBot       = QBotPlatformDriver(mode=1, ip=ipDriver)
    downCam      = QBotPlatformCSICamera(frameRate=frameRate, exposure = 39.0, gain=17.0)
    lidar        = QBotPlatformLidar()
    keyboard     = Keyboard()
    vision       = QBPVision()
    ranging      = QBPRanging()
    probe        = Probe(ip = ipHost)
    probe.add_display(imageSize = [800, 800, 1], scaling = True, scalingFactor= 2, name='Raw Image')
    probe.add_display(imageSize=[800, 800, 1], scaling=True, scalingFactor=2, name='Path')
    probe.add_plot(numMeasurements=1680, scaling=True, scalingFactor=4, name='Raw Lidar')
    probe.add_plot(numMeasurements=410, scaling=False, name='Plotting Lidar')

    line2SpdMap = vision.line_to_speed_map(sampleRate=sampleRate, saturation=75)
    next(line2SpdMap)
    startTime = time.time()
    time.sleep(0.5)
    pose = (0,0,0)
    world_frame = np.zeros((800, 800), np.uint8)

    world_frame_op = np.zeros((800, 800), np.uint8)

    path_map = np.zeros((800, 800), np.uint8)

    # pose_graph = ap.PoseGraph(
    #     dist_threshold=0.1,
    #     angle_threshold_deg=5
    # )
    prev_time = time.perf_counter()
    config = cont.PurePursuitConfig()

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

            # Keyboard Driver
            newkeyboard = keyboard.read()
            if newkeyboard:
                arm = keyboard.k_space
                lineFollow = keyboard.k_7
                if obstacle:
                    arm = 0
                keyboardCmd = keyboard.bodyCmd
                if keyboard.k_u:
                    noKill = False

            # Section C - toggle line following
            if not lineFollow:
                forSpd=keyboardCmd[0]
                turnSpd=keyboardCmd[1]

                commands = np.array([forSpd, turnSpd], dtype = np.float64) # robot spd command
    # --------------------------------- A star path and path controller -------------------------------
            else:
                # if map_generated == False:
                #     for n in pose_graph.nodes:
                #         pose = n.pose
                #         lidar_grid_opt = ap.lidarmapgrid(n.ranges, n.angles)
                #         world_frame_op = ap.place_local_into_global(lidar_grid_opt, n.pose, world_frame_op)
                #     cv2.imshow("Optimise_Frame", world_frame_op)
                #     map_generated = True
                #     if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit (optional)
                #         running = False

                if path_generated == False:
                    path_generated = True
                    res = 0.025
                    c_space = aas.compute_c_space(world_frame,3)
                    col_w = int(-(pose[0] / res) + 400)
                    row_w = int(-(pose[1] / res) + 400)

                    p1,p2 = ap.get_two_points(c_space)
                    # a star planner
                    # path,vv,nn = aas.search(c_space, (p1[1],p1[0]), (p2[1],p2[0]), "a_star", "euclidean")

                    planner = ds.DStarLite(start=(p1[1],p1[0]), goal= (p2[1],p2[0]), grid_map=c_space)

                    path = planner.get_path()
                    print('path:',path)
                    path_map = c_space.copy()
                    for n in path:
                        path_map [n] = 255
                    controller = cont.PurePursuitController(path, config)

                if ap.is_path_blocked(path,world_frame):
                    print('replanning path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                    res = 0.025
                    c_space = aas.compute_c_space(world_frame, 3)
                    col_w = int(-(pose[0] / res) + 400)
                    row_w = int(-(pose[1] / res) + 400)

                    path = planner.replan(start=(p1[1],p1[0]), new_map=c_space)

                    print('rep,...path:',path)
                    path_map = c_space.copy()
                    for n in path:
                        path_map[n] = 255
                    controller = cont.PurePursuitController(path, config)


                state = cont.RobotState(x = pose[0], y = pose[1], theta = pose[2])
                v_cmd, omega_cmd, wl, wr, lookahead = controller.compute_control(state)

                forSpd, turnSpd = v_cmd, omega_cmd
                if controller.is_goal_reached(state):
                    forSpd, turnSpd = 0, 0
                    print('Goal Reached')
                    path_generated = True
                print('speed:',forSpd,'turn:',turnSpd)
                commands = np.array([forSpd, turnSpd], dtype = np.float64) # robot spd command





            # QBot Hardware, read and update the wheel position
            newHIL = myQBot.read_write_std(timestamp = time.time() - startTime,
                                            arm = arm,
                                            commands = commands, userLED=obstacle)

            wp = myQBot.wheelPositions  # [L, R] rad
            gyro = myQBot.gyroscope  # [gx, gy, gz] rad/s
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




            if newHIL:
                timeHIL     = time.time()
                newDownCam  = downCam.read()
                newLidar    = lidar.read()

                if newDownCam:
                    counterDown += 1

                    # Section D - Image processing
                    # Undistort and resize the image
                    undistorted = vision.df_camera_undistort(downCam.imageData)
                    gray_sm = cv2.resize(undistorted, (320, 200))

                    #-------Replace the following line with your code---------#
                    # Subselect a part of the image and perform thresholding
                    binary = None

                    # Blob Detection via Connected Component Labeling
                    col, row, area = 0, 0, 0

                    # Section D.2 - Speed command from blob information
                  #  forSpd, turnSpd = line2SpdMap.send((col, 1, 0))
                    #---------------------------------------------------------#
                # Section E - Obstacles detection
                if newLidar:
                    counterLidar += 1

                    # Section E - Obstacles detection
                    rangesAdj, anglesAdj = ranging.adjust_and_subsample(lidar.distances, lidar.angles,1260,3)
                    rangesC, anglesC = ranging.correct_lidar([myQBot.LIDAR_POS_X,myQBot.LIDAR_POS_Y],rangesAdj, anglesAdj)

                    #---------------------------------------------------------#

                    # Section F - Obstacles detection
                    #-------lidar map grid generation---------#
                    minThreshold=0.2858
                    rangesP, anglesP, obstacle = ranging.detect_obstacle(rangesC, anglesC, forSpd, 0.5, turnSpd, 1, minThreshold, 10)
                    lidar_grid = ap.lidarmapgrid(rangesC, anglesC)


                    #----------------------------SLAM  1: GET POSE   put under if newlidar?? --------------------------#

                    timelimit += 1

                    if timelimit % 5  == 1:

                        now = time.perf_counter()
                        dt = now - prev_time
                        prev_time = now

                        # skip pose update on first iteration only
                        world_frame= ap.place_local_into_global(lidar_grid,pose,world_frame)

                #------------------------------------IM SHOW -----------------------------------
                        world_frame_location = world_frame.copy()
                        r,c = pose[0], pose[1]
                        r = -r/0.025 + 400
                        c = -c/0.025 + 400
                        robot_radius = 10
                        for dr in range(-robot_radius, robot_radius + 1):
                            for dc in range(-robot_radius, robot_radius + 1):
                                # Check Euclidean distance for circular robot [cite: 27, 29]
                                if np.sqrt(dr ** 2 + dc ** 2) <= robot_radius:
                                    nr, nc = r + dr, c + dc
                                    world_frame_location[int(nr), int(nc)] = 255


                        # cv2.imshow("World_Frame", world_frame_location)
                        # cv2.imshow("LiDAR", lidar_grid)
                        # cv2.imshow("path", path_map)
                        # if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit (optional)
                        #     running = False

# -----------------------       SLAM    edge and nodes          ------------------------------------



                    # if pose_graph.should_add_node(pose):
                    #     pose_graph.add_node(pose,rangesC, anglesC)
                    #     nodelimit += 1
                    #
                    #     if len(pose_graph.nodes) >= 2:
                    #         from_node = pose_graph.nodes[-2]
                    #         to_node = pose_graph.nodes[-1]
                    #
                    #         init = pose_graph.relative_pose_in_frame(from_node.pose, to_node.pose)
                    #
                    #         # Always add odom edge
                    #         pose_graph.add_edge(from_node.id, to_node.id, init,
                    #                       info_diag=(80, 80, 150), edge_type="odom")
                    #
                    #         P1 = pose_graph.scan_to_points(from_node.ranges, from_node.angles)
                    #         P2 = pose_graph.scan_to_points(to_node.ranges, to_node.angles)
                    #
                    #         #meas = relative pos after icp_2d optimise
                    #
                    #         meas, rmse, inliers = pose_graph.icp_2d(P1, P2, init=init)
                    #         score = max(0.0, min(1.0, inliers * np.exp(-rmse)))
                    #
                    #         #what if node after certain point diffuse heavily and every edge can not be added?
                    #         if inliers > 0.3 and rmse < 0.2:
                    #             pose_graph.add_edge(from_node.id, to_node.id, meas, info_diag=(200, 200, 400), edge_type="scan", score=score)
                    #             pose_graph.optimize_gtsam(robust_scan=("huber", 1.0), robust_loop=("huber", 1.0),
                    #                                       prior_sigmas=(1e-6, 1e-6, 1e-6), max_iters=50)
                    #
                    #             # lidar_grid_opt = ap.lidarmapgrid(pose_graph.nodes[-1].ranges,
                    #             #                                  pose_graph.nodes[-1].angles)
                    #             # world_frame_op = ap.place_local_into_global(lidar_grid_opt, pose_graph.nodes[-1].pose, world_frame_op)
                    #             # cv2.imshow("Optimise_Frame", world_frame_op)
                    #             # if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit (optional)
                    #             #     running = False
                    #
                    #         # ------------------------- LOOP CLOSURE---------------------------------------------
                    #         if nodelimit % 10 == 1:
                    #             candidate_list= ap.find_loop_candidate(pose_graph.nodes,to_node.id,1,10,10)
                    #             print('find loop candidate')
                    #             if candidate_list is not None:
                    #                 for c in candidate_list:
                    #                     _,c_id = c
                    #                     next_node = pose_graph.nodes[c_id]
                    #
                    #                     init = pose_graph.relative_pose_in_frame(to_node.pose, next_node.pose)
                    #
                    #                 # Always add odom edge
                    #                     pose_graph.add_edge(to_node.id, next_node.id, init,
                    #                                         info_diag=(80, 80, 150), edge_type="odom")
                    #
                    #                     P1 = pose_graph.scan_to_points(to_node.ranges, to_node.angles)
                    #                     P2 = pose_graph.scan_to_points(next_node.ranges, next_node.angles)
                    #
                    #                     # meas = relative pos after icp_2d optimise
                    #
                    #                     meas, rmse, inliers = pose_graph.icp_2d(P1, P2, init=init)
                    #                     score = max(0.0, min(1.0, inliers * np.exp(-rmse)))
                    #
                    #                 # what if node after certain point diffuse heavily and every edge can not be added?
                    #                     if inliers > 0.3 and rmse < 0.2:
                    #                         print('loop closure added')
                    #                         pose_graph.add_edge(from_node.id, to_node.id, meas, info_diag=(200, 200, 400),
                    #                                             edge_type="scan", score=score)
                    #                         pose_graph.optimize_gtsam(robust_scan=("huber", 1.0), robust_loop=("huber", 1.0),
                    #                                                   prior_sigmas=(1e-6, 1e-6, 1e-6), max_iters=50)
                    #                         print('pose optimised')
                    #                         break

                # # ------------------------------------------------------------------------------------------------------
                if counterDown%4 == 0:
                    sending = probe.send(name='Raw Image', imageData= world_frame_location)
                if counterLidar%2 == 0:
                    sending = probe.send(name='Raw Lidar', lidarData=(rangesC, anglesC))
                if counterLidar%2 == 1:
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
    keyboard.terminate()
    probe.terminate()
    cv2.destroyAllWindows()