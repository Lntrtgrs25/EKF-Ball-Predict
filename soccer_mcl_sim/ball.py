# import pygame
# import math

# class Ball:
#     def __init__(self, x=450.0, y=300.0): 
#         self.x = x
#         self.y = y
#         self.vx = 10.0 
#         self.vy = 10.0 
#         self.theta = 5.0        #coba cek lagi
#         self.friction = 0.99
#         self.radius = 10        #ini gada guna
#         self.is_draging = False #ini juga buat apa

#     def kick(self, target_x, target_y, power):
#         dx = target_x - self.x
#         dy = target_y - self.y
#         angle = math.atan2(dy, dx)
        
#         self.vx = math.cos(angle) * power           #velocity vector
#         self.vy = math.sin(angle) * power

#     def update(self, dt):
#         if not self.is_draging:
#             self.x += self.vx * dt                      #GLBB style-3
#             self.y += self.vy * dt
            
#             self.vx *= self.friction
#             self.vy *= self.friction
                
#             if abs(self.vx) < 0.1: self.vx = 0         #Epsilon Threshold
#             if abs(self.vy) < 0.1: self.vy = 0
        
#         if self.x < 0:
#             self.x = 0
#             self.vx = -self.vx
#         elif self.x > 900:
#             self.x = 900
#             self.vx = -self.vx

#         if self.y < 0:
#             self.y = 0
#             self.vy = -self.vy
#         elif self.y > 600:
#             self.y = 600
#             self.vy = -self.vy

#     def draw(self, screen, scale):
#         pygame.draw.circle(screen, (255, 49, 8), (int(self.x * scale), int(self.y * scale)), 7) #oranye like red

import pygame
import math

class Ball:
    def __init__(self, x=450.0, y=300.0): 
        self.x = x
        self.y = y
        self.vx = 0.0 
        self.vy = 0.0 
        self.theta = 0.0        # Arah hadap bola (untuk garis kuning)
        self.friction = 0.98    # Biar kerasa gesekan rumputnya
        self.radius = 10        # Sekarang ada gunanya buat deteksi klik
        self.is_dragging = False 

    def kick(self, power):
        # Sekarang kick pake self.theta (arah garis kuning)
        self.vx = math.cos(self.theta) * power
        self.vy = math.sin(self.theta) * power

    def update(self, dt):
        if self.is_dragging:
            # Kalau lagi ditarik, kecepatan harus nol biar gak "liar" pas dilepas
            self.vx = 0
            self.vy = 0
        else:
            # Fisika normal (GLBB style)
            self.x += self.vx * dt
            self.y += self.vy * dt
            
            # Gesekan
            self.vx *= self.friction
            self.vy *= self.friction
                
            # Stop kalau sudah sangat pelan
            if abs(self.vx) < 0.1: self.vx = 0
            if abs(self.vy) < 0.1: self.vy = 0
        
        # Pantulan Dinding
        # if self.x < self.radius:
        #     self.x = self.radius
        #     self.vx = -self.vx * 0.5 # Mantul tapi energinya hilang setengah
        # elif self.x > 900 - self.radius:
        #     self.x = 900 - self.radius
        #     self.vx = -self.vx * 0.5

        # if self.y < self.radius:
        #     self.y = self.radius
        #     self.vy = -self.vy * 0.5
        # elif self.y > 600 - self.radius:
        #     self.y = 600 - self.radius
        #     self.vy = -self.vy * 0.5

    def draw(self, screen, scale):
        # 1. Gambar badan bola
        pos = (int(self.x * scale), int(self.y * scale))
        pygame.draw.circle(screen, (255, 49, 8), pos, int(self.radius * scale))
        
        # 2. Gambar "Bibir" / Arah (Garis Kuning)
        # Garis ini menunjukkan kemana bola bakal meluncur pas di-kick
        line_len = 25
        end_x = self.x + math.cos(self.theta) * line_len
        end_y = self.y + math.sin(self.theta) * line_len
        pygame.draw.line(screen, (255, 255, 0), pos, (int(end_x * scale), int(end_y * scale)), 3)