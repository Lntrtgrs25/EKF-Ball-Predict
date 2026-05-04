import math

class KeeperMode:
    def __init__(self):
        self.goal_x = 15.0
        self.goal_y = 300.0
        self.radius_x = 80.0
        self.radius_y = 130.0
        self.target = None
        self.kp_pos = 5.0
        self.max_speed = 200.0
        self.kick_dist = 35.0
        self.kick_power = 1000.0

    def update(self, robot, ball, ekf_pos, ekf_vel, ball_detected, dt):
        if not ball_detected:
            robot.vx = 0.0
            robot.vy = 0.0
            self.target = None
            return

        # Update orientation to face ball
        dx_ball = ball.x - robot.x
        dy_ball = ball.y - robot.y
        dist_robot_ball = math.hypot(dx_ball, dy_ball)
        robot.theta = math.atan2(dy_ball, dx_ball)

        # Get ball position 
        bx = ekf_pos[0] if ekf_pos else ball.x
        by = ekf_pos[1] if ekf_pos else ball.y

        # Calculate target on ellipse
        delta_x = bx - self.goal_x
        delta_y = by - self.goal_y

        if delta_x == 0 and delta_y == 0:
            k = 0.0
        else:
            pembagi = (delta_x**2 / self.radius_x**2) + (delta_y**2 / self.radius_y**2)
            k = 1.0 / math.sqrt(pembagi)

        target_x = self.goal_x + k * delta_x
        target_y = self.goal_y + k * delta_y
        self.target = (target_x, target_y)

        # Movement control (Global to Local)
        v_global_x = target_x - robot.x
        v_global_y = target_y - robot.y

        v_local_x = v_global_x * math.cos(robot.theta) + v_global_y * math.sin(robot.theta)
        v_local_y = -v_global_x * math.sin(robot.theta) + v_global_y * math.cos(robot.theta)

        vx_cmd = self.kp_pos * v_local_x
        vy_cmd = self.kp_pos * v_local_y

        speed_mag = math.hypot(vx_cmd, vy_cmd)
        if speed_mag > self.max_speed:
            vx_cmd = (vx_cmd / speed_mag) * self.max_speed
            vy_cmd = (vy_cmd / speed_mag) * self.max_speed

        robot.vx = vx_cmd
        robot.vy = vy_cmd

        # Kicking Logic
        if dist_robot_ball < self.kick_dist:
            b_vx = ekf_vel[0] if ekf_vel else ball.vx
            b_vy = ekf_vel[1] if ekf_vel else ball.vy
            
            if math.hypot(b_vx, b_vy) > 10.0:
                # Arah datang bola (velocity vector)
                incoming_angle = math.atan2(b_vy, b_vx)
                # Arah balik bola (opposite of incoming) + 15 derajat
                ball.theta = incoming_angle + math.pi + math.radians(15)
            else:
                ball.theta = robot.theta
            
            ball.kick(self.kick_power)
            print(f"[KEEPER] Kick! Distance: {dist_robot_ball:.1f}, Angle: {math.degrees(ball.theta):.1f}")
