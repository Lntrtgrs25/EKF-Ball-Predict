import pygame
import math

class Ball:
    def __init__(self, x=450.0, y=300.0): 
        self.x = x
        self.y = y
        self.vx = 0.0 
        self.vy = 0.0 
        self.theta = 0.0        
        self.friction = 0.98    
        self.radius = 10        
        self.is_dragging = False 

    def kick(self, power):
        self.vx = math.cos(self.theta) * power
        self.vy = math.sin(self.theta) * power

    def update(self, dt):
        if self.is_dragging:
            self.vx = 0
            self.vy = 0
        else:
            self.x += self.vx * dt
            self.y += self.vy * dt
            
            self.vx *= self.friction
            self.vy *= self.friction
                
            if abs(self.vx) < 0.1: 
                self.vx = 0
            if abs(self.vy) < 0.1: 
                self.vy = 0

    def draw(self, screen, scale):
        bx = int(self.x * scale)
        by = int(self.y * scale)
        pygame.draw.circle(screen, (255, 49, 8), (bx, by), int(self.radius * scale))
        
        line = 25
        end_x = self.x + math.cos(self.theta) * line
        end_y = self.y + math.sin(self.theta) * line
        pygame.draw.line(screen, (255, 255, 0), (bx, by), (int(end_x * scale), int(end_y * scale)), 3)