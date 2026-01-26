# vision.py
import math
import random
import pygame


class VisionSensor:
    def __init__(
        self,
        fov_deg=78.0,
        max_range=350.0,
        n_rays=61,
        range_noise_std=3.0,
        bearing_noise_std_deg=1.0,
        range_noise_gain=0.02,
        bearing_noise_gain_deg=0.01,
    ):
        self.fov = math.radians(fov_deg)
        self.max_range = max_range
        self.n_rays = n_rays

        self.range_noise_std = range_noise_std
        self.bearing_noise_std = math.radians(bearing_noise_std_deg)

        self.range_noise_gain = range_noise_gain
        self.bearing_noise_gain = math.radians(bearing_noise_gain_deg)

    def sense(self, robot, field, enable_noise=True):
        observations = []

        all_landmarks = []
        for p in field.landmarks_L:
            all_landmarks.append(("L-Intersection", p))
        for p in field.landmarks_T:
            all_landmarks.append(("T-Intersection", p))
        for p in field.landmarks_X:
            all_landmarks.append(("X-Intersection", p))
        for p in field.landmarks_goalpost:
            all_landmarks.append(("goalpost", p))

        for lm_type, (lx, ly) in all_landmarks:
            dx = lx - robot.x
            dy = ly - robot.y

            true_range = math.hypot(dx, dy)
            if true_range > self.max_range:
                continue

            bearing = math.atan2(dy, dx) - robot.theta
            bearing = self._wrap_angle(bearing)

            if abs(bearing) > self.fov / 2:
                continue

            if enable_noise:
                sigma_r = self.range_noise_std + self.range_noise_gain * true_range
                sigma_b = self.bearing_noise_std + self.bearing_noise_gain * true_range

                noisy_range = true_range + random.gauss(0.0, sigma_r)
                noisy_bearing = bearing + random.gauss(0.0, sigma_b)
            else:
                noisy_range = true_range
                noisy_bearing = bearing

            observations.append(
                {
                    "type": lm_type,
                    "true_pos": (lx, ly),
                    "true_range": true_range,
                    "true_bearing": bearing,
                    "range": noisy_range,
                    "bearing": noisy_bearing,
                }
            )

        return observations


    def draw_fov(self, screen, robot, scale=1.0):
        """
        Visualize FoV cone and rays
        """
        ORANGE = (255, 180, 50)
        GRAY = (200, 200, 200)

        cx = int(robot.x * scale)
        cy = int(robot.y * scale)

        start_angle = robot.theta - self.fov / 2
        end_angle = robot.theta + self.fov / 2

        # FoV boundary
        for angle in [start_angle, end_angle]:
            ex = cx + int(self.max_range * scale * math.cos(angle))
            ey = cy + int(self.max_range * scale * math.sin(angle))
            pygame.draw.line(screen, ORANGE, (cx, cy), (ex, ey), 2)

        # Rays
        for i in range(self.n_rays):
            a = start_angle + i * (self.fov / (self.n_rays - 1))
            ex = cx + int(self.max_range * scale * math.cos(a))
            ey = cy + int(self.max_range * scale * math.sin(a))
            pygame.draw.line(screen, GRAY, (cx, cy), (ex, ey), 1)

    def draw_measurements(self, screen, robot, observations, scale=1.0):
        """
        Draw:
        - line robot -> true landmark
        - line robot -> noisy measurement
        - noisy landmark point
        """

        TRUE_LINE = (180, 180, 180)     # abu-abu
        MEAS_LINE = (255, 100, 100)     # merah
        MEAS_POINT = (255, 60, 60)

        rx = robot.x
        ry = robot.y
        rtheta = robot.theta

        cx = int(rx * scale)
        cy = int(ry * scale)

        for obs in observations:
            # ---- noisy measurement position ----
            meas_x = rx + obs["range"] * math.cos(obs["bearing"] + rtheta)
            meas_y = ry + obs["range"] * math.sin(obs["bearing"] + rtheta)

            mx = int(meas_x * scale)
            my = int(meas_y * scale)

            # line to noisy estimate
            pygame.draw.line(
                screen,
                MEAS_LINE,
                (cx, cy),
                (mx, my),
                2,
            )

            # noisy landmark point
            pygame.draw.circle(screen, MEAS_POINT, (mx, my), 6)


    @staticmethod
    def _wrap_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def set_fov(self, deg):
        self.fov = math.radians(max(10.0, min(180.0, deg)))

    def set_range(self, r):
        self.max_range = max(100.0, min(2000.0, r))

    def set_noise(self, r_std, b_std_deg, r_gain=None, b_gain_deg=None):
        self.range_noise_std = max(0.0, r_std)
        self.bearing_noise_std = math.radians(max(0.0, b_std_deg))

        if r_gain is not None:
            self.range_noise_gain = max(0.0, r_gain)
        if b_gain_deg is not None:
            self.bearing_noise_gain = math.radians(max(0.0, b_gain_deg))

