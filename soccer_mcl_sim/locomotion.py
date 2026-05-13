import math
import collections

class Locomotion:
    def __init__(self, 
                 max_linear_vel=250.0, 
                 max_angular_vel=3.0, 
                 max_accel=150.0, 
                 max_alpha=5.0, 
                 step_delay=0.2):
        """
        Pseudo-Physics Locomotion for Bipedal Robot.
        Meniru batasan mekanis robot humanoid nyata.
        
        :param max_linear_vel: Batas kecepatan linear maksimum (px/s).
        :param max_angular_vel: Batas kecepatan rotasi maksimum (rad/s).
        :param max_accel: Batas akselerasi linear (px/s^2).
        :param max_alpha: Batas akselerasi angular (rad/s^2).
        :param step_delay: Jeda waktu (detik) sebelum koordinat mulai berubah.
        """
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.max_accel = max_accel
        self.max_alpha = max_alpha
        self.step_delay = step_delay
        
        # Kecepatan saat ini (real velocity) yang sedang dijalankan robot
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        
        # Buffer untuk simulasi jeda langkah (Step Delay)
        # Menyimpan tuple (timestamp, vx_target, vy_target, omega_target)
        self.command_buffer = collections.deque()
        
        # Target kecepatan yang baru keluar dari delay (sebelum diproses akselerasi)
        self.delayed_target_vx = 0.0
        self.delayed_target_vy = 0.0
        self.delayed_target_omega = 0.0

    def set_command(self, vx, vy, omega, current_time):
        """
        Menerima perintah gerak (target) dan memasukkannya ke antrian delay.
        """
        # 1. Velocity Clamping (Batas Kecepatan Maksimum)
        mag = math.hypot(vx, vy)
        if mag > self.max_linear_vel:
            vx = (vx / mag) * self.max_linear_vel
            vy = (vy / mag) * self.max_linear_vel
        
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))
        
        # 2. Masukkan ke buffer dengan timestamp sekarang
        self.command_buffer.append((current_time, vx, vy, omega))

    def update(self, x, y, theta, dt, current_time):
        """
        Update posisi dan orientasi robot berdasarkan fisika buatan.
        """
        # 3. Step Delay Logic
        # Ambil perintah terbaru yang sudah melewati masa tunggu (delay)
        while self.command_buffer and (current_time - self.command_buffer[0][0]) >= self.step_delay:
            _, self.delayed_target_vx, self.delayed_target_vy, self.delayed_target_omega = self.command_buffer.popleft()

        # 4. Inertia / Acceleration Limit (Logika Low-Pass Filter / Accel Cap)
        # Menghitung selisih kecepatan untuk mencapai target
        dvx = self.delayed_target_vx - self.vx
        dvy = self.delayed_target_vy - self.vy
        domega = self.delayed_target_omega - self.omega
        
        # Batasi akselerasi linear
        d_linear = math.hypot(dvx, dvy)
        if d_linear > self.max_accel * dt and d_linear > 1e-6:
            self.vx += (dvx / d_linear) * self.max_accel * dt
            self.vy += (dvy / d_linear) * self.max_accel * dt
        else:
            # Jika selisih kecil, langsung set ke target agar tidak overshoot/bergetar
            self.vx = self.delayed_target_vx
            self.vy = self.delayed_target_vy
            
        # Batasi akselerasi angular (rotasi)
        if abs(domega) > self.max_alpha * dt:
            self.omega += math.copysign(self.max_alpha * dt, domega)
        else:
            self.omega = self.delayed_target_omega

        # 5. Omnidirectional Kinematics Update (2D)
        # Robot bipedal bergerak relatif terhadap arah hadapnya (theta)
        # vx = maju/mundur, vy = geser samping
        dx_local = self.vx * dt
        dy_local = self.vy * dt
        
        # Transformasi dari koordinat lokal robot ke koordinat global lapangan
        dx_global = dx_local * math.cos(theta) - dy_local * math.sin(theta)
        dy_global = dx_local * math.sin(theta) + dy_local * math.cos(theta)
        
        new_x = x + dx_global
        new_y = y + dy_global
        new_theta = theta + self.omega * dt
        
        return new_x, new_y, new_theta
