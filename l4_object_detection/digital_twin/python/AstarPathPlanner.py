import numpy as np
import time
# import gtsam
import cv2
from dataclasses import dataclass

def get_two_points(map_img):
    points = []
    img = map_img.copy()

    def click_event(event, x, y, flags, param):
        nonlocal points, img
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
            points.append((x, y))
            cv2.circle(img, (x, y), 5, (255, 255, 255), -1)
            cv2.imshow("map", img)

            if len(points) == 2:
                cv2.destroyAllWindows()

    cv2.imshow("map", img)
    cv2.setMouseCallback("map", click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print('two points:', points)
    return points

def is_path_blocked(path, grid, radius=1, blocked_values={255}):

    rows = 800
    cols = 800

    for x, y in path:
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy

                # treat out of bounds as blocked
                if nx < 0 or ny < 0 or ny >= rows or nx >= cols:
                    return True

                if grid[nx][ny] in blocked_values:
                    return True
    return False

def lidarmapgrid(distances,angles):

    offset = np.int16(400 / 2)
    frameSize = 400
    pixelsPerMeter = 40

    valid = distances < 5
    distances = distances[valid]
    angles = angles[valid]

    image_lidar = np.zeros((frameSize,frameSize), dtype=np.uint8)

    x = np.int16(np.clip(offset - pixelsPerMeter * distances * np.cos(angles), 0, frameSize - 1))
    y = np.int16(np.clip(offset - pixelsPerMeter * distances * np.sin(angles), 0, frameSize - 1))
    image_lidar[x, y] = np.uint8(255)
    return image_lidar

def find_loop_candidate(nodes,cur_id,dis_range,skip_num,candidate_num):

    candidate_id_end = cur_id-skip_num
    candidate_cur = 0

    if (candidate_id_end) <= 0:
        return []

    distance = []

    for id in range(candidate_id_end):
        x = nodes[id].pose[0]
        y = nodes[id].pose[1]
        xj = nodes[cur_id].pose[0]
        yj = nodes[cur_id].pose[1]
        d = np.sqrt((x - xj)**2 + (y - yj)**2)

        if d < dis_range:
            distance.append((d,id))
            candidate_cur += 1

        if candidate_cur >= candidate_num:
            break

    distance.sort(key=lambda x: x[0])

    return distance

def wrap_to_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi

def update_pose_diffdrive(
    pose,
    wheel_positions,
    prev_wheel_positions,
    dt,
    wheel_radius,
    wheel_base,
    gyroscope=None,
    gyro_weight=0,
):
    """
    Differential-drive odometry pose update using wheel positions (rad),
    optionally fused with gyro yaw rate (rad/s).

    Args:
        pose: (x, y, theta) current pose in meters/radians (world frame).
        wheel_positions: np.array([left_rad, right_rad]) current wheel angles (rad).
        prev_wheel_positions: np.array([left_rad, right_rad]) previous wheel angles (rad).
        dt: time step seconds.
        wheel_radius: wheel radius in meters.
        wheel_base: distance between wheels in meters.
        gyroscope: optional np.array([gx, gy, gz]) rad/s. Uses gz for yaw rate.
        gyro_weight: 0..1. 0 = wheel-only heading, 1 = gyro-only heading.

    Returns:
        new_pose: (x_new, y_new, theta_new)
        new_prev_wheel_positions: updated previous wheel positions for next call
    """
    x, y, theta = pose

    # Guard
    if dt <= 0:
        return (x, y, theta), wheel_positions.copy()

    # Wheel angle increments (rad)
    d_left_rad  = float(wheel_positions[0] - prev_wheel_positions[0])
    d_right_rad = float(wheel_positions[1] - prev_wheel_positions[1])

    # Convert to wheel travel distances (m)
    d_left_m  = wheel_radius * d_left_rad
    d_right_m = wheel_radius * d_right_rad

    # Differential drive motion
    d_s     = 0.5 * (d_left_m + d_right_m)           # forward distance (m)
    dtheta_wheels = (d_right_m - d_left_m) / wheel_base  # yaw change (rad)

    # Optional gyro yaw integration
    if gyroscope is not None:
        gz = float(gyroscope[2])  # yaw rate rad/s
        dtheta_gyro = gz * dt
        dtheta = (1.0 - gyro_weight) * dtheta_wheels + gyro_weight * dtheta_gyro
    else:
        dtheta = dtheta_wheels

    # Use midpoint heading for better accuracy on curves
    theta_mid = theta + 0.5 * dtheta

    # Update position in world frame (x forward, y left)
    x_new = x + d_s * np.cos(theta_mid)
    y_new = y + d_s * np.sin(theta_mid)
    theta_new = wrap_to_pi(theta + dtheta)


    return (x_new, y_new, theta_new), wheel_positions.copy()


def place_local_into_global(
    local_grid,
    pose,            # (x,y,theta) in meters/radians
    global_grid,
    res = 0.025
):
    """
    Copies occupied cells from local robot-centered grid
    into global world grid.
    """
    origin_x = 400
    origin_y = 400
    H, W = local_grid.shape
    gh, gw = global_grid.shape



    xr, yr, th = pose
    th = -th + np.pi/2



    c = np.cos(th)
    s = np.sin(th)

    center_r = H // 2
    center_c = W // 2

    for r in range(H):
        for col in range(W):

            # only place occupied cells
            if local_grid[r, col] != 255:
                continue

            # local coords (meters)
            x_local = (col - center_c) * res
            y_local = (r - center_r) * res



            # world coords
            xw = xr + x_local * c - y_local * s
            yw = yr - x_local * s - y_local * c

            local_grid[r, col] = 255

            # world → global grid indices
            col_w = int(-(xw / res) +400)
            row_w = int(-(yw  / res) + 400)



            if 0 <= row_w < gh and 0 <= col_w < gw:
                global_grid[col_w, row_w] += 1
            if global_grid[col_w, row_w]>= 3:
                global_grid[col_w, row_w] = 255

    return global_grid



@dataclass
class Node:
    id: int
    pose: tuple  # (x, y, theta)
    ranges: np.ndarray  # LiDAR distances
    angles: np.ndarray  # LiDAR angles
    timestamp: float

@dataclass
class Edge:
    from_id: int
    to_id: int
    measurement: tuple      # (dx, dy, dtheta) in from_id frame
    information: np.ndarray # 3x3 weight matrix
    edge_type: str          # "odom" / "scan" / "loop"

class PoseGraph:
    def __init__(self,
                 dist_threshold=0.1,  # meters
                 angle_threshold_deg=5):  # degrees

        self.nodes = []
        self.edges = []
        self.next_id = 0
        self.dist_threshold = dist_threshold
        self.angle_threshold = np.deg2rad(angle_threshold_deg)

    def should_add_node(self, current_pose):
        """
        Decide whether movement is large enough
        to create a new node.
        """
        if len(self.nodes) == 0:
            return True

        last_pose = self.nodes[-1].pose

        dx = current_pose[0] - last_pose[0]
        dy = current_pose[1] - last_pose[1]
        dtrans = np.sqrt(dx * dx + dy * dy)

        dtheta = wrap_to_pi(current_pose[2] - last_pose[2])

        if dtrans > self.dist_threshold:
            return True

        if abs(dtheta) > self.angle_threshold:
            return True

        return False


    def add_node(self, pose, ranges, angles):
        """
        Store a new graph node.
        """
        node = Node(
            id=self.next_id,
            pose=pose,
            ranges=ranges.copy(),
            angles=angles.copy(),
            timestamp=time.time()
        )

        self.nodes.append(node)
        self.next_id += 1

        print(f"Added node {node.id} at pose {node.pose}")


    def relative_pose_in_frame(self,pose1, pose2):

    # Relative transform from pose1 -> pose2 expressed in pose1 frame.
    # pose = (x, y, theta) in meters/radians.
    # Returns (dx_local, dy_local, dtheta).

        x1, y1, th1 = pose1
        x2, y2, th2 = pose2

        dx = x2 - x1
        dy = y2 - y1
        dth = wrap_to_pi(th2 - th1)

        c = np.cos(th1)
        s = np.sin(th1)

        dx_local =  c * dx + s * dy
        dy_local = -s * dx + c * dy

        return (dx_local, dy_local, dth)


    def add_odometry_edge_last(self, info_diag=(100.0, 100.0, 200.0)):
        """Add odom edge between the last two nodes using their poses."""
        if len(self.nodes) < 2:
            return None

        n1 = self.nodes[-2]
        n2 = self.nodes[-1]

        meas = relative_pose_in_frame(n1.pose, n2.pose)
        info = np.diag(np.array(info_diag, dtype=float))

        e = Edge(
            from_id=n1.id,
            to_id=n2.id,
            measurement=meas,
            information=info,
            edge_type="odom",
        )
        self.edges.append(e)
        return e

    def add_edge(self, from_id, to_id, measurement, info_diag=(200.0, 200.0, 400.0), edge_type="scan", score=1.0):
        """
        Generic edge add (use for scan-matching and loop closure).
        measurement: (dx, dy, dtheta) expressed in from_id frame.
        score scales confidence (e.g. scan match score).
        """
        score = float(score)
        if score <= 0:
            score = 1e-6

        w = np.array(info_diag, dtype=float) * score
        info = np.diag(w)

        e = Edge(
            from_id=int(from_id),
            to_id=int(to_id),
            measurement=(float(measurement[0]), float(measurement[1]), float(measurement[2])),
            information=info,
            edge_type=edge_type,
        )
        self.edges.append(e)
        return e



    def scan_to_points(self,ranges, angles, r_min=0.05, r_max=8.0, step=2):
        ranges = np.asarray(ranges)[::step]
        angles = np.asarray(angles)[::step]
        ok = np.isfinite(ranges) & (ranges >= r_min) & (ranges <= r_max)
        r = ranges[ok]
        a = angles[ok]
        return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)  # (N,2)

    def transform_points(self,P, dx, dy, dtheta):
        c, s = np.cos(dtheta), np.sin(dtheta)
        R = np.array([[c, -s],
                      [s, c]])
        return (P @ R.T) + np.array([dx, dy])

    def best_fit_transform(self,A, B):
        muA = A.mean(axis=0);
        muB = B.mean(axis=0)
        AA = A - muA;
        BB = B - muB
        H = AA.T @ BB
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T
        t = muB - (R @ muA)
        return R, t

    def icp_2d(self, ref_pts, cur_pts, init=(0.0, 0.0, 0.0), max_iters=20, max_corr_dist=0.4, tol=1e-4):
        dx, dy, dth = init
        last_rmse = None
        inliers = np.zeros(len(cur_pts), dtype=bool)

        for _ in range(max_iters):
            cur_w = self.transform_points(cur_pts, dx, dy, dth)

            d2 = ((cur_w[:, None, :] - ref_pts[None, :, :]) ** 2).sum(axis=2)
            nn_idx = np.argmin(d2, axis=1)
            nn_d = np.sqrt(d2[np.arange(len(cur_w)), nn_idx])

            inliers = nn_d < max_corr_dist
            if inliers.sum() < 20:
                break

            A = cur_w[inliers]
            B = ref_pts[nn_idx[inliers]]

            R, t = self.best_fit_transform(A, B)
            delta_th = np.arctan2(R[1, 0], R[0, 0])

            dx, dy = (np.array([dx, dy]) @ R.T + t)
            dth = dth + delta_th

            A_aligned = A @ R.T + t
            rmse = float(np.sqrt(np.mean(np.sum((A_aligned - B) ** 2, axis=1))))

            if last_rmse is not None and abs(last_rmse - rmse) < tol:
                last_rmse = rmse
                break
            last_rmse = rmse

        inlier_ratio = float(inliers.sum()) / float(len(cur_pts)) if len(cur_pts) else 0.0
        dth = (dth + np.pi) % (2 * np.pi) - np.pi
        return (float(dx), float(dy), float(dth)), (last_rmse if last_rmse is not None else 1e9), inlier_ratio

    def diag_info_to_sigmas(self, info):
        info = np.asarray(info, dtype=float)

        if info.ndim == 2:
            info = np.diag(info)

        if info.shape != (3,):
            raise ValueError(f"Expected 3 diagonal information values, got shape {info.shape}")

        return 1.0 / np.sqrt(np.maximum(info, 1e-12))

    def make_noise_model(self,info_diag, robust=None):
        base = gtsam.noiseModel.Diagonal.Sigmas(self.diag_info_to_sigmas(info_diag))
        if robust is None:
            return base

        # robust = ("huber", k) or ("cauchy", k)
        name, k = robust
        name = name.lower()
        if name == "huber":
            m = gtsam.noiseModel.mEstimator.Huber(k)
        elif name == "cauchy":
            m = gtsam.noiseModel.mEstimator.Cauchy(k)
        else:
            raise ValueError(f"Unknown robust loss: {robust}")
        return gtsam.noiseModel.Robust.Create(m, base)

    def optimize_gtsam(self, robust_scan=("huber", 1.0), robust_loop=("huber", 1.0),
                       prior_sigmas=(1e-6, 1e-6, 1e-6), max_iters=50):
        """
        Batch pose-graph optimization in SE(2) using GTSAM.
        Updates self.nodes[i].pose in-place.

        robust_scan/robust_loop:
          - None for no robust loss
          - ("huber", k) or ("cauchy", k)
        """
        if len(self.nodes) == 0:
            return

        # Build factor graph
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()

        # Add initial estimates for every node
        for n in self.nodes:
            x, y, th = n.pose
            initial.insert(int(n.id), gtsam.Pose2(float(x), float(y), float(th)))

        # Anchor the graph with a strong prior on the first node
        first_id = int(self.nodes[0].id)
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(prior_sigmas, dtype=float))
        graph.add(gtsam.PriorFactorPose2(first_id, gtsam.Pose2(0.0, 0.0, 0.0), prior_noise))

        # Add Between factors for each edge
        for e in self.edges:
            i = int(e.from_id)
            j = int(e.to_id)
            dx, dy, dth = e.measurement
            meas = gtsam.Pose2(float(dx), float(dy), float(dth))

            # Choose noise model (optionally robust)
            if getattr(e, "edge_type", "") == "scan":
                noise = self.make_noise_model(e.information, robust=robust_scan)
            elif getattr(e, "edge_type", "") == "loop":
                noise = self.make_noise_model(e.information, robust=robust_loop)
            else:
                noise = self.make_noise_model(e.information, robust=None)

            graph.add(gtsam.BetweenFactorPose2(i, j, meas, noise))

        # Optimize
        params = gtsam.LevenbergMarquardtParams()
        params.setMaxIterations(int(max_iters))

        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
        result = optimizer.optimize()

        # Write back optimized poses
        for n in self.nodes:
            p = result.atPose2(int(n.id))
            n.pose = (float(p.x()), float(p.y()), float(p.theta()))

























