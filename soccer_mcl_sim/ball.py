# import pygame
# import math

# class Ball:
#     def __init__(self, x=450.0, y=300.0):
#         self.x = x
#         self.y = y
#         self.vx = 150.0  # Kecepatan awal
#         self.vy = 100.0
#         self.friction = 0.99  # Biar bola bisa berhenti perlahan

#     def update(self, dt):
#         self.x += self.vx * dt
#         self.y += self.vy * dt
        
#         # Pantulan dinding (Lp: 900, Wp: 600)
#         if self.x <= 0 or self.x >= 900: self.vx *= -1
#         if self.y <= 0 or self.y >= 600: self.vy *= -1
        
#         # Efek gesekan rumput
#         self.vx *= self.friction
#         self.vy *= self.friction

#     def draw(self, screen, scale):
#         pygame.draw.circle(screen, (255, 49, 8), (int(self.x * scale), int(self.y * scale)), 7)

import pygame
import math

class Ball:
    def __init__(self, x=450.0, y=300.0): 
        self.x = x
        self.y = y
        self.vx = 10.0 
        self.vy = 10.0 
        self.friction = 0.99 

    def kick(self, target_x, target_y, power):
        dx = target_x - self.x
        dy = target_y - self.y
        angle = math.atan2(dy, dx)
        
        self.vx = math.cos(angle) * power
        self.vy = math.sin(angle) * power

    def update(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        self.vx *= self.friction
        self.vy *= self.friction
        
        if abs(self.vx) < 0.1: self.vx = 0
        if abs(self.vy) < 0.1: self.vy = 0

    def draw(self, screen, scale):
        pygame.draw.circle(screen, (255, 49, 8), (int(self.x * scale), int(self.y * scale)), 7)