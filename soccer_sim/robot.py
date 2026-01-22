# robot.py
import math
import pygame

class Robot:
    def __init__(self):
        self.x = 450.0
        self.y = 300.0
        self.theta = 0.0  # rad

        self.v = 0.0
        self.omega = 0.0

    def update(self, dt):
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
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
