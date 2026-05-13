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
        self.scan_speed = 0.4               # Kecepatan putar saat scanning (rad/s)
        self.scan_limit = math.radians(60)  # Batas tengok kanan-kiri (contoh: 60 derajat)
        self.scan_dir = 1.0

    def update(self, robot, ball, ekf_pos, ekf_vel, ball_detected, dt, locomotion_system):
        if not ball_detected:
            # Hentikan pergerakan translasi (maju/menyamping) secara perlahan (momentum deselerasi)
            # Kita turunkan nilainya bertahap agar tidak mengerem mendadak
            robot.vx *= 0.8
            robot.vy *= 0.8
            self.target = None
            
            # --- LOGIKA SCANNING KANAN-KIRI ---
            # Asumsi default gawang ada di kiri (X=15), jadi robot menghadap tengah lapangan (sudut 0 radian)
            # Normalisasi sudut saat ini ke rentang -pi hingga pi
            current_angle = (robot.theta + math.pi) % (2 * math.pi) - math.pi
            
            # Cek apakah orientasi robot sudah melebihi batas tengokan, jika ya, putar balik!
            if current_angle > self.scan_limit:
                self.scan_dir = -1.0  # Balik ke kanan
            elif current_angle < -self.scan_limit:
                self.scan_dir = 1.0   # Balik ke kiri
                
            # Set kecepatan putar robot
            robot.omega = self.scan_dir * self.scan_speed
            return

        # Get ball position dari EKF atau real ball 
        bx = ekf_pos[0] if ekf_pos else ball.x
        by = ekf_pos[1] if ekf_pos else ball.y

        # Hitung jarak murni untuk logika tendangan
        dist_robot_ball = math.hypot(ball.x - robot.x, ball.y - robot.y)

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

        # Eksekusi fungsi pergerakan keeper yang fokus pada strafing
        locomotion_system.move_keeper_strafing(
            robot=robot, 
            target_x=target_x, 
            target_y=target_y, 
            face_x=bx,   # Wajah selalu mengikuti bola / EKF
            face_y=by, 
            dt=dt,
            stop_distance=5.0
        )
        # ------------------------------------------------

        if self.kick_timer > 0.0:
            self.kick_timer -= dt

        # 2. Eksekusi tendangan HANYA jika jarak memenuhi DAN kaki sudah siap (timer = 0)
        if dist_robot_ball < self.kick_dist and self.kick_timer <= 0.0:
            
            # Arah tendangan alami: Mengikuti garis vektor dari pusat robot menembus pusat bola
            # Hal ini membuat tendangan bervariasi tergantung robot menyentuh sisi kiri/kanan bola
            kick_angle = math.atan2(ball.y - robot.y, ball.x - robot.x)
            
            # Tambahkan sedikit "noise" mekanis (kisaran -5 hingga 5 derajat) 
            # agar tendangan tidak selalu 100% presisi layaknya mesin CNC
            noise_angle = math.radians(random.uniform(-5.0, 5.0))
            final_kick_angle = kick_angle + noise_angle
            
            # Hitung kecepatan datangnya bola untuk transfer momentum
            b_vx = ekf_vel[0] if ekf_vel else ball.vx
            b_vy = ekf_vel[1] if ekf_vel else ball.vy
            ball_speed = math.hypot(b_vx, b_vy)
            
            # Makin cepat bola mengarah ke robot, makin besar power pantulannya (Transfer Energi)
            actual_power = self.kick_power + (ball_speed * 0.4)
            actual_power = min(actual_power, 1600.0)  # Batasi agar bola tidak lenyap dari layar

            # Eksekusi tendangan ke bola
            ball.theta = final_kick_angle
            ball.kick(actual_power)
            
            # Efek Recoil: Robot akan terdorong sedikit ke belakang setelah menendang
            # Ini mensimulasikan momentum langkah penyeimbang setelah mengayunkan kaki
            robot.vx -= math.cos(final_kick_angle) * 35.0
            robot.vy -= math.sin(final_kick_angle) * 35.0

            # Kunci sistem menendang agar kaki kembali ke posisi awal (kuda-kuda)
            self.kick_timer = self.kick_cooldown
            
            print(f"[KEEPER] CLEARED! Pwr: {actual_power:.1f}, Ang: {math.degrees(final_kick_angle):.1f} deg")