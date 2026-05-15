import math
import random

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
        self.kick_timer = 0.0
        self.kick_cooldown = 1.2
        self.scan_speed = 0.4               
        self.scan_limit = math.radians(60)  
        self.scan_dir = 1.0
        
        # --- Pemilihan Mode Lintasan ---
        # Bisa diubah menjadi 'ellipse' atau 'vertical'
        self.trajectory_mode = 'vertical' 

    def update(self, robot, ball, ekf_pos, ekf_vel, ball_detected, dt, locomotion_system):
        if not ball_detected:
            # Hentikan pergerakan translasi secara perlahan
            robot.vx *= 0.8
            robot.vy *= 0.8
            self.target = None
            
            # Logika scanning kanan-kiri
            current_angle = (robot.theta + math.pi) % (2 * math.pi) - math.pi
            
            if current_angle > self.scan_limit:
                self.scan_dir = -1.0  
            elif current_angle < -self.scan_limit:
                self.scan_dir = 1.0   
                
            robot.omega = self.scan_dir * self.scan_speed
            return

        # Get ball position dari EKF atau real ball 
        bx = ekf_pos[0] if ekf_pos else ball.x
        by = ekf_pos[1] if ekf_pos else ball.y

        # Hitung jarak murni untuk logika tendangan
        dist_robot_ball = math.hypot(ball.x - robot.x, ball.y - robot.y)

        if self.trajectory_mode == 'ellipse':
            delta_x = bx - self.goal_x
            delta_y = by - self.goal_y

            if delta_x == 0 and delta_y == 0:
                k = 0.0
            else:
                pembagi = (delta_x**2 / self.radius_x**2) + (delta_y**2 / self.radius_y**2)
                k = min(1.0, 1.0 / math.sqrt(pembagi))

            target_x = self.goal_x + k * delta_x
            target_y = self.goal_y + k * delta_y
            
        elif self.trajectory_mode == 'vertical':
            line_x = self.goal_x + 10.0
            
            if bx < line_x:
                target_x = max(self.goal_x, bx)
            else:
                target_x = line_x
                
            min_y = self.goal_y - self.radius_y
            max_y = self.goal_y + self.radius_y
            target_y = max(min_y, min(max_y, by))

        self.target = (target_x, target_y)

        locomotion_system.move_keeper_strafing(
            robot=robot, 
            target_x=target_x, 
            target_y=target_y, 
            face_x=bx,   
            face_y=by, 
            dt=dt,
            stop_distance=5.0
        )

        if self.kick_timer > 0.0:
            self.kick_timer -= dt

        if dist_robot_ball < self.kick_dist and self.kick_timer <= 0.0:
            
            kick_angle = math.atan2(ball.y - robot.y, ball.x - robot.x)
            noise_angle = math.radians(random.uniform(-5.0, 5.0))
            final_kick_angle = kick_angle + noise_angle
            
            if math.cos(final_kick_angle) < 0.0:
                if ball.y > self.goal_y:
                    final_kick_angle = math.radians(70.0) + noise_angle
                else:
                    final_kick_angle = math.radians(-70.0) + noise_angle

            b_vx = ekf_vel[0] if ekf_vel else ball.vx
            b_vy = ekf_vel[1] if ekf_vel else ball.vy
            ball_speed = math.hypot(b_vx, b_vy)
            
            actual_power = self.kick_power + (ball_speed * 0.4)
            actual_power = min(actual_power, 1600.0)

            ball.theta = final_kick_angle
            ball.kick(actual_power)
            
            robot.vx -= math.cos(final_kick_angle) * 35.0
            robot.vy -= math.sin(final_kick_angle) * 35.0

            self.kick_timer = self.kick_cooldown
            
            print(f"[KEEPER] CLEARED! Pwr: {actual_power:.1f}, Ang: {math.degrees(final_kick_angle):.1f} deg")