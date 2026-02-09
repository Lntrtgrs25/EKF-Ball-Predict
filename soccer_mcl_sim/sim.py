# sim.py
import random
import rclpy
from rclpy.node import Node
import pygame
import time
import math

from .world import Field
from .robot import Robot
from .vision import VisionSensor
from .ui import InputField, Button, Toggle
from .ball import Ball   

from gyakuenki_interfaces.msg import ProjectedObjects, ProjectedObject
from aruku_interfaces.msg import Point2
from kansei_interfaces.msg import Status, Axis
# from basho_interfaces.msg import Particles

class SoccerSim(Node):
    def __init__(self):
        super().__init__("soccer_mcl_sim")
        pygame.init()

        self.scale = 1.0
        self.padding = 0
        self.field = Field()
        self.robot = Robot()

        self.vision = VisionSensor(
            fov_deg=78.0,
            max_range=450.0,
            n_rays=61,
            range_noise_std=3.0,
            bearing_noise_std_deg=1.0,
            range_noise_gain=0.02,
            bearing_noise_gain_deg=0.01
        )

        self.ball = Ball() 
        self.pub_projected = self.create_publisher(  
            ProjectedObjects,
            "/gyakuenki_cpp/projected_objects",
            10
        )
        self.last_time = time.time()  

        PANEL_W = 220
        self.screen = pygame.display.set_mode(
            (
                int(self.field.length * self.scale) + PANEL_W + self.padding,
                int(self.field.width * self.scale) + self.padding * 2,
            )
        )

        pygame.display.set_caption("Soccer MCL Simulator")

        self.last_time = time.time()
        self.font = pygame.font.SysFont("monospace", 16)

        # self.pub_tuning = self.create_publisher(Point2, "/ball/ekf_tuning", 10)

        # ---------------- UI Layout ----------------
        self.panel_x = int(self.field.length * self.scale) + self.padding + 10
        y = 20

        self.buttons = []

        def add_button(label, cb):
            nonlocal y
            self.buttons.append(Button(self.panel_x, y, 180, 28, label, cb))
            y += 36

        add_button("FoV +", lambda: self.vision.set_fov(
            math.degrees(self.vision.fov) + 5))
        add_button("FoV -", lambda: self.vision.set_fov(
            math.degrees(self.vision.fov) - 5))

        add_button("Range +", lambda: self.vision.set_range(
            self.vision.max_range + 50))
        add_button("Range -", lambda: self.vision.set_range(
            self.vision.max_range - 50))

        add_button("Noise +", lambda: self.vision.set_noise(
            self.vision.range_noise_std + 1,
            math.degrees(self.vision.bearing_noise_std) + 0.5))
        add_button("Noise -", lambda: self.vision.set_noise(
            self.vision.range_noise_std - 1,
            math.degrees(self.vision.bearing_noise_std) - 0.5))
        #Kasih button buat tuning Q sama R
        # add_button("Q +", lambda: self.vision)

        y += 10
        self.toggle_rays = Toggle(self.panel_x, y, "Show rays", True)
        y += 30
        self.toggle_noise = Toggle(self.panel_x, y, "Enable noise", True)
        y += 50

        # ---- Interactive kidnap state ----
        self.kidnap_pos = None      # (x, y)
        self.kidnap_theta = 0.0
        self.is_dragging = False
        self.just_kidnapped = False

        # ---------- Kidnap Inputs (X, Y, Theta) ----------
        self.kidnap_x_box = InputField(
            self.panel_x, y, 100, 28,
            "X", self.robot.x, 0, self.field.length
        )
        y += 52

        self.kidnap_y_box = InputField(
            self.panel_x, y, 100, 28,
            "Y", self.robot.y, 0, self.field.width
        )
        y += 52

        self.kidnap_theta_box = InputField(
            self.panel_x, y, 100, 28,
            "Theta (deg)", 0, -180, 180
        )
        y += 40

        self.kidnap_button = Button(
            self.panel_x, y, 160, 36, "KIDNAP", self._kidnap_cb
        )

        # ----------- ROS2 Publishers -----------
        self.pub_projected = self.create_publisher(
            ProjectedObjects,
            "/gyakuenki_cpp/projected_objects",
            10
        )

        self.pub_delta = self.create_publisher(
            Point2,
            "walking/delta_position",
            10
        )

        self.pub_imu = self.create_publisher(
            Status,
            "measurement/status",
            10
        )

        # ----------- ROS2 Subscriber -----------
        # self.sub_particles = self.create_subscription(
        #     Particles,
        #     "localization/particles",
        #     self._particles_cb,
        #     10
        # )

        self.sub_odometry = self.create_subscription(
            Point2,
            "walking/set_odometry",
            self._odometry_cb,
            10
        )

        self.sub_imu = self.create_subscription(
            Status,
            "measurement/status",
            self._imu_cb,
            10
        )

        self.sub_ekf = self.create_subscription(
            Point2,
            "/ball/filtered_pos",
            self._ekf_callback,
            10
        )

        # self.particles_msg = None
        self.prev_x = self.robot.x
        self.prev_y = self.robot.y
        self.ekf_ball_pos = None 

    # ---------------- Callback ----------------
    def _kidnap_cb(self):
        if self.kidnap_pos is None:
            return

        x = self.kidnap_x_box.get_value()
        y = self.kidnap_y_box.get_value()
        theta_deg = self.kidnap_theta_box.get_value()
        theta = math.radians(theta_deg)

        self.robot.kidnap(x, y, theta)
        self.prev_x = x
        self.prev_y = y
        self.just_kidnapped = True

        print(f"[KIDNAP] Robot moved to ({x:.1f}, {y:.1f}, {theta_deg:.1f} deg)")

    # def _particles_cb(self, msg):
    #     self.particles_msg = msg

    def _odometry_cb(self, msg):
        self.robot.belief_x = msg.x
        self.robot.belief_y = msg.y

    def _imu_cb(self, msg):
        yaw_deg = msg.orientation.yaw
        self.robot.belief_theta = math.radians(yaw_deg)

    def _ekf_callback(self, msg):
        self.ekf_ball_pos = (msg.x, msg.y)

    # def _apply_tuning_cb(self):
    #     msg = Point2()
    #     msg.x = self.tuning_q_box.get_value() # Kita anggap ini Q
    #     msg.y = self.tuning_r_box.get_value() # Kita anggap ini R
    #     self.pub_tuning.publish(msg)
    #     print(f"[TUNING] Q set to: {msg.x}, R set to: {msg.y}")

    # ---------------- Main Loop ----------------
    def step(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        self.ball.update(dt)

        mouse_pos = pygame.mouse.get_pos()          #for mouse koordinat
        mx = mouse_pos[0] / self.scale
        my = mouse_pos[1] / self.scale

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

            for b in self.buttons:
                b.handle_event(event)

            self.toggle_rays.handle_event(event)
            self.toggle_noise.handle_event(event)
            self.kidnap_x_box.handle_event(event)
            self.kidnap_y_box.handle_event(event)
            self.kidnap_theta_box.handle_event(event)
            self.kidnap_button.handle_event(event)
            #kalo mau tambah tuning q r disini

            # -------- Mouse kidnap --------
            if event.type == pygame.MOUSEBUTTONDOWN:
                # mx, my = event.pos
                dist_to_ball = math.hypot(mx - self.ball.x, my - self.ball.y)

                # if mx < self.field.length * self.scale:
                #     self.kidnap_pos = (mx / self.scale, my / self.scale)
                #     self.is_dragging = True

                if event.button == 1:
                    if dist_to_ball < 20:
                        self.ball.is_dragging = True
                    else:
                        self.kidnap_pos = (mx, my)
                        self.is_dragging = True
    
                elif event.button == 3:
                        self.ball.kick(1300.0)

            if event.type == pygame.MOUSEBUTTONUP:
                self.ball.is_dragging = False
                # self.is_dragging = False

            if event.type == pygame.MOUSEMOTION:
                if self.ball.is_dragging:
                    self.ball.x = mx
                    self.ball.y = my
                elif self.is_dragging and self.kidnap_pos is not None:
                    x0, y0 = self.kidnap_pos

                    dx = (mx / self.scale) - x0
                    dy = (my / self.scale) - y0

                    self.kidnap_theta = math.atan2(dy, dx)
                    self.kidnap_x_box.set_value(x0)
                    self.kidnap_y_box.set_value(y0)
                    self.kidnap_theta_box.set_value(
                        math.degrees(self.kidnap_theta)
                    )
                else:
                    dist_to_ball = math.hypot(mx - self.ball.x, my - self.ball.y)
                    if dist_to_ball < 100:
                        self.ball.theta = math.atan2(my - self.ball.y, mx - self.ball.x)  # Optional: remove if not needed for ball behavior

        # ---- Autonomous Robot Behavior (Facing Ball and Interception) ----
        # Calculate distance and angle to ball
        dx = self.ball.x - self.robot.x
        dy = self.ball.y - self.robot.y
        dist_robot_ball = math.hypot(dx, dy)
        angle_to_ball = math.atan2(dy, dx)
        
        # Check if ball is in FOV and range
        rel_angle = (angle_to_ball - self.robot.theta + math.pi) % (2 * math.pi) - math.pi
        if dist_robot_ball < self.vision.max_range and abs(rel_angle) < (self.vision.fov / 2):
            # Robot automatically faces the ball
            self.robot.theta = angle_to_ball

        # # Prediction Raycasting (Anticipate Goal)
        # # Use EKF position if available
        # bx = self.ekf_ball_pos[0] if self.ekf_ball_pos else self.ball.x
        # by = self.ekf_ball_pos[1] if self.ekf_ball_pos else self.ball.y
        
        # if self.ball.vx < -10:  # If ball is moving toward left goal (x=0)
        #     x_gawang = 50.0 
        #     # Raycasting: predict impact y using ball's movement direction
        #     ball_move_dir = math.atan2(self.ball.vy, self.ball.vx)
        #     y_impact = by + (x_gawang - bx) * math.tan(ball_move_dir)
            
        #     # Draw prediction line (Cyan)
        #     pygame.draw.line(self.screen, (0, 255, 255), (bx, by), (x_gawang, y_impact), 1)

        #     # If predicted to enter goal area (y=200-400) and close enough, intercept
        #     if 200 < y_impact < 400 and (bx - x_gawang) < 250:
        #         if self.robot.y < y_impact - 5:
        #             self.robot.vy = 120.0
        #         elif self.robot.y > y_impact + 5:
        #             self.robot.vy = -120.0
        #         else:
        #             self.robot.vy = 0.0
        #     else:
        #         self.robot.vy = 0.0  # Stop if safe
        # else:
        #     self.robot.vy = 0.0

        # Keyboard control
        keys = pygame.key.get_pressed()
        self.robot.vx = 0.0
        self.robot.vy = 0.0
        self.robot.omega = 0.0

        # if keys[pygame.K_w]:
        #     self.robot.vx = 100.0
        # if keys[pygame.K_s]:
        #     self.robot.vx = -100.0
        # if keys[pygame.K_l]:
        #     self.robot.vy = 100.0
        # if keys[pygame.K_j]:
        #     self.robot.vy = -100.0
        if keys[pygame.K_d]:
            self.robot.omega = 2.0
        if keys[pygame.K_a]:
            self.robot.omega = -2.0

        self.robot.update(dt)

        ODO_ALPHA_DIST = 0.05
        ODO_SIGMA_MIN = 0.5
        IMU_SIGMA_DEG = 1.0

        if self.just_kidnapped:
            dx_true = 0.0
            dy_true = 0.0
            self.just_kidnapped = False
        else:
            dx_true = self.robot.x - self.prev_x
            dy_true = self.robot.y - self.prev_y

        dist = math.hypot(dx_true, dy_true)

        # Noise std
        sigma_d = ODO_ALPHA_DIST * abs(dist) + ODO_SIGMA_MIN

        # Noisy odometry
        dx_noisy = dx_true + random.gauss(0, sigma_d)
        dy_noisy = dy_true + random.gauss(0, sigma_d)

        msg = Point2()
        msg.x = dx_noisy
        msg.y = dy_noisy
        self.pub_delta.publish(msg)

        if abs(msg.x) > 10.0 or abs(msg.y) > 10.0:
            print("[DEBUG] Large odometry:", msg.x, msg.y)

        self.prev_x = self.robot.x
        self.prev_y = self.robot.y

        # ---- IMU NOISE MODEL ----
        yaw_deg_true = math.degrees(self.robot.theta)
        yaw_noisy = (
            yaw_deg_true +
            random.gauss(0, IMU_SIGMA_DEG)
        )

        while yaw_noisy > 180:
            yaw_noisy -= 360
        while yaw_noisy < -180:
            yaw_noisy += 360

        imu = Status()
        imu.is_calibrated = True
        imu.orientation = Axis()
        imu.orientation.roll = 0.0
        imu.orientation.pitch = 0.0
        imu.orientation.yaw = yaw_noisy

        self.pub_imu.publish(imu)

        # ---------------- Drawing ----------------
        self.screen.fill((15, 20, 30))  # background

        self.field.draw(
            self.screen,
            self.scale,
            offset_x=self.padding,
            offset_y=self.padding
        )

        # self.draw_particles()

        self.robot.draw(self.screen, self.scale)
        # self.draw_estimated_position()
        self.robot.draw_belief(self.screen, self.scale)

        # ---- Draw Ball (Original) ----
        self.ball.draw(self.screen, self.scale)

        # ---- Draw Ball (EKF Result - Yellow Circle) ----
        if self.ekf_ball_pos is not None:
            ex = int(self.ekf_ball_pos[0] * self.scale)
            ey = int(self.ekf_ball_pos[1] * self.scale)
            
            pygame.draw.circle(self.screen, (255, 255, 0), (ex, ey), 8, 2) # Lingkaran kuning
            pygame.draw.circle(self.screen, (255, 255, 0), (ex, ey), 2)    # Titik tengah

        # ---- Draw kidnap preview ----
        if self.kidnap_pos is not None:
            x, y = self.kidnap_pos
            px = int(x * self.scale)
            py = int(y * self.scale)

            pygame.draw.circle(self.screen, (255, 200, 0), (px, py), 6)

            L = 40
            ex = px + int(L * math.cos(self.kidnap_theta))
            ey = py + int(L * math.sin(self.kidnap_theta))
            pygame.draw.line(
                self.screen, (255, 200, 0),
                (px, py), (ex, ey), 3
            )

        observations = self.vision.sense(
            self.robot, self.field,
            enable_noise=self.toggle_noise.value
        )

        # ---- Publish projected objects ----
        po_msg = ProjectedObjects()

        for obs in observations:
            o = ProjectedObject()

            r = obs["range"]
            b = obs["bearing"]

            rel_x = r * math.cos(b)
            rel_y = r * math.sin(b)

            o.position.x = rel_x * 0.01
            o.position.y = rel_y * -0.01
            o.position.z = 0.0

            o.confidence = 1.0
            o.label = obs["type"]

            po_msg.projected_objects.append(o)

        ball_po = ProjectedObject()
        noise_x = random.gauss(0, 5.0) 
        noise_y = random.gauss(0, 5.0)
        ball_po.position.x = (self.ball.x + noise_x) * 0.01 
        ball_po.position.y = (self.ball.y + noise_y) * -0.01 
        ball_po.label = "ball"
        po_msg.projected_objects.append(ball_po)

        self.pub_projected.publish(po_msg)

        self.vision.draw_fov(self.screen, self.robot, self.scale)
        if self.toggle_rays.value:
            self.vision.draw_measurements(
                self.screen, self.robot, observations, self.scale
            )

        # ---------------- UI Panel (dark, not grey) ----------------
        panel_rect = pygame.Rect(
            self.panel_x - 10, 0,
            220, self.screen.get_height()
        )
        pygame.draw.rect(self.screen, (220, 220, 220), panel_rect)
        pygame.draw.rect(self.screen, (180, 180, 180), panel_rect, 2)

        for b in self.buttons:
            b.draw(self.screen, self.font)

        self.toggle_rays.draw(self.screen, self.font)
        self.toggle_noise.draw(self.screen, self.font)
        self.kidnap_x_box.draw(self.screen, self.font)
        self.kidnap_y_box.draw(self.screen, self.font)
        self.kidnap_theta_box.draw(self.screen, self.font)
        self.kidnap_button.draw(self.screen, self.font)

        # info text
        info_y = panel_rect.bottom - 90
        lines = [
            f"FoV   : {math.degrees(self.vision.fov):.1f} deg",
            f"Range : {self.vision.max_range:.0f}",
            f"NoiseR: {self.vision.range_noise_std:.1f}",
            f"NoiseB: {math.degrees(self.vision.bearing_noise_std):.1f}",
        ]
        for l in lines:
            txt = self.font.render(l, True, (0, 0, 0))
            self.screen.blit(txt, (self.panel_x, info_y))
            info_y += 18

        pygame.display.flip()
        return True

    def weight_to_alpha(self, w, k=6.0):
        return 1.0 - math.exp(-k * w)

    # def draw_particles(self):
    #     if self.particles_msg is None:
    #         return

    #     for p in self.particles_msg.particles:
    #         x = int(p.x * self.scale)
    #         y = int(p.y * self.scale)

    #         alpha = self.weight_to_alpha(p.weight)
    #         alpha = max(0.05, min(alpha, 1.0))

    #         r = 3
    #         s = pygame.Surface((r*2, r*2), pygame.SRCALPHA)
    #         # color = (0, 100, 200, int(255 * alpha))
    #         color = (0, 100, 200, int(255))
    #         pygame.draw.circle(s, color, (r, r), r)
    #         self.screen.blit(s, (x-r, y-r))

    # def draw_estimated_position(self):
    #     if self.particles_msg is None:
    #         return

    #     ep = self.particles_msg.estimated_position
    #     x = int(ep.x * self.scale)
    #     y = int(ep.y * self.scale)
    #     theta = self.robot.belief_theta

    #     R = 14

    #     pygame.draw.circle(self.screen, (0, 200, 0), (x, y), R, 2)

    #     hx = x + R * math.cos(theta)
    #     hy = y + R * math.sin(theta)
    #     pygame.draw.line(self.screen, (0, 200, 0), (x, y), (hx, hy), 2)

def main():
    rclpy.init()
    sim = SoccerSim()

    running = True
    while running and rclpy.ok():
        rclpy.spin_once(sim, timeout_sec=0.033)
        running = sim.step()

    sim.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == "__main__":
    main()