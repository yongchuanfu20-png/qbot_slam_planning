import math
from dataclasses import dataclass
from typing import List, Tuple
import time

from numpy.ma.core import maximum


@dataclass
class RobotState:
    x: float          # meters, origin at map center
    y: float          # meters, origin at map center
    theta: float      # rad


@dataclass
class PurePursuitConfig:
    # Map settings
    map_resolution: float = 0.025                 # meters per cell
    map_center_cell: Tuple[float, float] = (400, 400)

    # Set this True if your grid y-axis points downward (image-style coordinates)
    # Set this False if your grid y-axis already points upward
    invert_grid_y: bool = True

    # Pure pursuit settings
    lookahead_distance: float = 0.2       # meters
    target_speed: float = 0.3                   # m/s

    # Robot settings
    wheelbase: float = 0.3928                     # meters
    wheel_radius: float = 3.5*0.0254/2                   # meters

    # Limits
    max_linear_speed: float = 0.50                # m/s
    max_angular_speed: float = 1           # rad/s
    max_wheel_angular_speed: float = 15.0         # rad/s

    # Goal behavior
    goal_tolerance: float = 0.3               # meters
    slowdown_radius: float = 0.40                 # meters
    min_speed_near_goal: float = 0.1      # m/s

    # pd control
    kp = 0.6
    kd = 1


class PurePursuitController:
    """
    Input path: grid cells, e.g. [(gx1, gy1), (gx2, gy2), ...]
    Input robot pose: meters from map center, i.e. state.x, state.y
    Controller internally converts path grid -> meters.
    """

    def __init__(self, path_grid: List[Tuple[float, float]], config: PurePursuitConfig):
        if len(path_grid) < 2:
            raise ValueError("Path must contain at least 2 waypoints.")
        if config.lookahead_distance <= 0:
            raise ValueError("lookahead_distance must be > 0.")
        if config.map_resolution <= 0:
            raise ValueError("map_resolution must be > 0.")

        self.config = config
        self.path_grid = path_grid
        self.path_m = [self.grid_to_world(gx, gy) for gx, gy in path_grid]
        self.last_closest_index = 0
        # use in pd contro
        self.prev_time = time.perf_counter()
        self.prev_error = 0

    def grid_to_world(self, gx: float, gy: float) -> Tuple[float, float]:
        """
        Convert grid cell coordinate -> meters relative to map center.
        Center cell [400, 400] maps to (0, 0) meters.

        If invert_grid_y=True:
            grid y increasing downward -> world y increasing upward
        """
        cx, cy = self.config.map_center_cell
        res = self.config.map_resolution

        x_m = (cx - gx) * res

        if self.config.invert_grid_y:
            y_m = (cy - gy) * res
        else:
            y_m = (gy - cy) * res

        return x_m, y_m

    @staticmethod
    def _clamp(value: float, vmin: float, vmax: float) -> float:
        return max(vmin, min(value, vmax))

    @staticmethod
    def _distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def _find_closest_waypoint_index(self, state: RobotState) -> int:
        best_idx = self.last_closest_index
        best_dist = float("inf")

        for i in range(self.last_closest_index, len(self.path_m)):
            d = math.hypot(self.path_m[i][0] - state.x, self.path_m[i][1] - state.y)
            if d < best_dist:
                best_dist = d
                best_idx = i

        self.last_closest_index = best_idx
        return best_idx

    def _find_lookahead_point(self, closest_idx: int) -> Tuple[float, float]:
        """
        Returns lookahead point in meters.
        """
        L = self.config.lookahead_distance
        acc_dist = 0.0
        current_point = self.path_m[closest_idx]

        for i in range(closest_idx, len(self.path_m) - 1):
            p1 = self.path_m[i]
            p2 = self.path_m[i + 1]
            seg_len = self._distance(p1, p2)

            if acc_dist + seg_len >= L:
                remaining = L - acc_dist
                if seg_len < 1e-9:
                    return p2

                ratio = remaining / seg_len
                x = p1[0] + ratio * (p2[0] - p1[0])
                y = p1[1] + ratio * (p2[1] - p1[1])
                return (x, y)

            acc_dist += seg_len
            current_point = p2

        return current_point

    @staticmethod
    def _transform_to_robot_frame(
        state: RobotState, target_point: Tuple[float, float]
    ) -> Tuple[float, float]:
        dx = target_point[0] - state.x
        dy = target_point[1] - state.y

        x_r = math.cos(state.theta) * dx + math.sin(state.theta) * dy
        y_r = -math.sin(state.theta) * dx + math.cos(state.theta) * dy

        print('control value transfrom to robot frame x,y:',x_r, y_r)
        return x_r, y_r

    def _compute_speed(self, state: RobotState) -> float:
        goal = self.path_m[-1]
        dist_to_goal = math.hypot(goal[0] - state.x, goal[1] - state.y)

        if dist_to_goal < self.config.slowdown_radius:
            scale = dist_to_goal / max(self.config.slowdown_radius, 1e-9)
            speed = max(self.config.min_speed_near_goal, self.config.target_speed * scale)
        else:
            speed = self.config.target_speed

        return self._clamp(speed, -self.config.max_linear_speed, self.config.max_linear_speed)

    def is_goal_reached(self, state: RobotState) -> bool:
        goal = self.path_m[-1]
        print('goal:',goal)
        print('robot state:',state)
        dist_to_goal = math.hypot(goal[0] - state.x, goal[1] - state.y)
        print('goal distance:',dist_to_goal)
        return dist_to_goal <= self.config.goal_tolerance

    def compute_control(
        self, state: RobotState
    ) -> Tuple[float, float, float, float, Tuple[float, float]]:
        """
        Returns:
            v_cmd            [m/s]
            omega_cmd        [rad/s]
            wl               [rad/s]
            wr               [rad/s]
            lookahead_point  [meters]
        """
        if self.is_goal_reached(state):
            return 0.0, 0.0, 0.0, 0.0, self.path_m[-1]

        closest_idx = self._find_closest_waypoint_index(state)
        lookahead_point = self._find_lookahead_point(closest_idx)

        x_r, y_r = self._transform_to_robot_frame(state, lookahead_point)

        L = max(self.config.lookahead_distance, 1e-9)

        # pd contro

        current_time = time.perf_counter()
        dt = current_time - self.prev_time
        self.prev_time = current_time


        curvature =  2* y_r / (L * L)

        error = y_r
        d_error = (error - self.prev_error) / dt

        print('selferror,error,dt,y_r:',self.prev_error,error,dt,y_r)

        curvature = curvature*self.config.kp + self.config.kd * d_error
        self.prev_error = error


        v_cmd = self._compute_speed(state)
        omega_cmd = v_cmd * curvature
        omega_cmd = self._clamp(
            omega_cmd,
            -self.config.max_angular_speed,
            self.config.max_angular_speed,
        )

        b = self.config.wheelbase
        r = self.config.wheel_radius

        v_right = v_cmd + 0.5 * b * omega_cmd
        v_left = v_cmd - 0.5 * b * omega_cmd

        wr = v_right / r
        wl = v_left / r

        wr = self._clamp(wr, -self.config.max_wheel_angular_speed, self.config.max_wheel_angular_speed)
        wl = self._clamp(wl, -self.config.max_wheel_angular_speed, self.config.max_wheel_angular_speed)

        return v_cmd, omega_cmd, wl, wr, lookahead_point

