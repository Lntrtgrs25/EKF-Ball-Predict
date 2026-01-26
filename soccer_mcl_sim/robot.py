# robot.py
import math
import pygame

class Robot:
    def __init__(self):
        self.x = 450.0
        self.y = 300.0
        self.theta = 0.0  # rad

        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

    def update(self, dt):
        dx = self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)
        dy = self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)

        self.x += dx * dt
        self.y += dy * dt
        self.theta += self.omega * dt

    def draw(self, screen, scale=1.0):
        px = int(self.x * scale)
        py = int(self.y * scale)

        pygame.draw.circle(screen, (0, 0, 0), (px, py), 10)

        hx = px + 15 * math.cos(self.theta)
        hy = py + 15 * math.sin(self.theta)
        pygame.draw.line(screen, (255, 0, 0), (px, py), (hx, hy), 2)

    def kidnap(self, x, y, theta=None):
        self.x = x
        self.y = y
        if theta is not None:
            self.theta = theta
