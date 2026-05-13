import math
import collections

def keisan_map(val, in_min, in_max, out_min, out_max):
    if in_min < in_max:
        val = max(in_min, min(in_max, val))
    else:
        val = max(in_max, min(in_min, val))
        
    if in_max == in_min: return out_min
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class Locomotion:
    def __init__(self, 
                 max_linear_vel=250.0, 
                 max_angular_vel=3.0, 
                 max_accel=150.0, 
                 max_alpha=5.0, 
                 step_delay=0.2,
                 move_max_x=100.0,
                 move_max_y=180.0,  # Prioritas kecepatan menyamping lebih tinggi
                 move_max_a=2.0):
        """
        Pseudo-Physics Locomotion for Bipedal Robot.
        Meniru batasan mekanis robot humanoid nyata.
        """
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.max_accel = max_accel
        self.max_alpha = max_alpha
        self.step_delay = step_delay
        
        # Mapping limits
        self.move_max_x = move_max_x
        self.move_max_y = move_max_y
        self.move_max_a = move_max_a
        
        # Kecepatan saat ini (real velocity) yang sedang dijalankan robot
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        
        # Buffer untuk simulasi jeda langkah (Step Delay)
        self.command_buffer = collections.deque()
        
        # Target kecepatan yang baru keluar dari delay
        self.delayed_target_vx = 0.0
        self.delayed_target_vy = 0.0
        self.delayed_target_omega = 0.0

    def move_keeper_strafing(self, robot, target_x, target_y, face_x, face_y, dt, stop_distance=5.0):
        """
        Menggerakkan robot ke target (x, y) dengan fokus strafing, 
        sambil memastikan orientasi badan selalu menghadap titik bola (face_x, face_y),
        dengan tambahan limitasi akselerasi fisik (momentum).
        """
        # 1. Hitung error orientasi (Harus menghadap bola/titik prediksi)
        dx_face = face_x - robot.x
        dy_face = face_y - robot.y
        target_theta = math.atan2(dy_face, dx_face)
        
        delta_theta_rad = target_theta - robot.theta
        delta_theta_rad = (delta_theta_rad + math.pi) % (2 * math.pi) - math.pi
        delta_direction = math.degrees(delta_theta_rad)
        
        # Kecepatan putar TARGET (omega) dipetakan dari error sudut
        target_omega = keisan_map(delta_direction, -25.0, 25.0, -self.move_max_a, self.move_max_a)

        # 2. Hitung jarak dan vektor menuju target posisi (elips gawang)
        dx_target = target_x - robot.x
        dy_target = target_y - robot.y
        target_distance = math.hypot(dx_target, dy_target)

        # 3. Hitung kecepatan linear TARGET (X dan Y lokal)
        if target_distance < stop_distance:
            target_vx = 0.0
            target_vy = 0.0
        else:
            # Transformasi Global ke Lokal
            local_x = dx_target * math.cos(robot.theta) + dy_target * math.sin(robot.theta)
            local_y = -dx_target * math.sin(robot.theta) + dy_target * math.cos(robot.theta)

            target_vx = keisan_map(abs(local_x), 0.0, 80.0, 0.0, self.move_max_x)
            target_vx = target_vx if local_x > 0 else -target_vx
            
            target_vy = keisan_map(abs(local_y), 0.0, 80.0, 0.0, self.move_max_y)
            target_vy = target_vy if local_y > 0 else -target_vy

            # Penalti kecepatan gerak jika robot sedang melenceng jauh dari arah hadap bola
            if abs(delta_direction) > 30.0:
                target_vx *= 0.3
                target_vy *= 0.3

        # 4. TERAPKAN ACCELERATION LIMIT (Mencegah "Teleport")
        # Mengatur perlambatan/percepatan linear (Maju & Menyamping)
        dvx = target_vx - robot.vx
        dvy = target_vy - robot.vy
        
        accel_req = math.hypot(dvx, dvy) / dt if dt > 0 else 0.0
        
        if accel_req > self.max_accel:
            # Limit akselerasi agar momentum terbangun perlahan
            scale = self.max_accel / accel_req
            dvx *= scale
            dvy *= scale
            
        robot.vx += dvx
        robot.vy += dvy
        
        # Mengatur perlambatan/percepatan putaran badan (Rotasi)
        domega = target_omega - robot.omega
        alpha_req = abs(domega) / dt if dt > 0 else 0.0
        
        if alpha_req > self.max_alpha:
            domega = math.copysign(self.max_alpha * dt, domega)
            
        robot.omega += domega

        return target_distance < stop_distance

    # [PERTahankan def set_command(self, vx, vy, omega, current_time): HINGGA AKHIR FILE TANPA PERUBAHAN]
    # [PERTahankan def update(self, x, y, theta, dt, current_time): HINGGA AKHIR FILE TANPA PERUBAHAN]