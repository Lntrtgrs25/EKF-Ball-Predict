# world.py
import pygame
import math


class Field:
    def __init__(self):
        self.width = 600
        self.length = 900

        self.landmarks_L = [
            (0.0, 0.0),
            (0.0, 600.0),
            (100.0, 50.0),
            (100.0, 550.0),
            (900.0, 0.0),
            (900.0, 600.0),
            (800.0, 50.0),
            (800.0, 550.0),
        ]

        self.landmarks_T = [
            (0.0, 50.0),
            (0.0, 550.0),
            (450.0, 0.0),
            (450.0, 600.0),
            (900.0, 50.0),
            (900.0, 550.0),
        ]

        self.landmarks_X = [
            (210, 300),
            (690, 300),
            (450, 300),
            (450, 375),
            (450, 225),
        ]

        self.landmarks_goalpost = [
            (0.0, 170.0),
            (0.0, 430.0),
            (900.0, 170.0),
            (900.0, 430.0),
        ]

    def draw(self, screen, scale=1.0, offset_x=0, offset_y=0):
        # Colors
        GREEN = (40, 150, 40)
        WHITE = (245, 245, 245)
        RED = (220, 50, 50)
        BLUE = (50, 100, 220)
        YELLOW = (240, 200, 40)
        BLACK = (20, 20, 20)

        FIELD_W = int(self.width * scale)
        FIELD_L = int(self.length * scale)

        LINE_W = 4
        MARK_SIZE = 6

        # Background grass (full screen not overwritten)
        pygame.draw.rect(
            screen,
            GREEN,
            pygame.Rect(offset_x, offset_y, FIELD_L, FIELD_W),
        )

        # =============================
        # Field lines
        # =============================

        # Outer boundary
        pygame.draw.rect(
            screen,
            WHITE,
            pygame.Rect(offset_x, offset_y, FIELD_L, FIELD_W),
            LINE_W,
        )

        # Center line
        pygame.draw.line(
            screen,
            WHITE,
            (offset_x + FIELD_L // 2, offset_y),
            (offset_x + FIELD_L // 2, offset_y + FIELD_W),
            LINE_W,
        )

        # Center circle
        pygame.draw.circle(
            screen,
            WHITE,
            (offset_x + FIELD_L // 2, offset_y + FIELD_W // 2),
            int(75 * scale),
            LINE_W,
        )

        goal_area_w = int(100 * scale)
        goal_area_h = int(500 * scale)

        # Left goal area
        pygame.draw.rect(
            screen,
            WHITE,
            pygame.Rect(
                offset_x,
                offset_y + (FIELD_W - goal_area_h) // 2,
                goal_area_w,
                goal_area_h,
            ),
            LINE_W,
        )

        # Right goal area
        pygame.draw.rect(
            screen,
            WHITE,
            pygame.Rect(
                offset_x + FIELD_L - goal_area_w,
                offset_y + (FIELD_W - goal_area_h) // 2,
                goal_area_w,
                goal_area_h,
            ),
            LINE_W,
        )

        # Penalty points
        pygame.draw.circle(
            screen,
            WHITE,
            (offset_x + int(210 * scale), offset_y + FIELD_W // 2),
            8,
        )

        pygame.draw.circle(
            screen,
            WHITE,
            (offset_x + int(690 * scale), offset_y + FIELD_W // 2),
            8,
        )

        # =============================
        # Landmarks
        # =============================

        # # L landmarks
        # for x, y in self.landmarks_L:
        #     px = offset_x + int(x * scale)
        #     py = offset_y + int(y * scale)
        #     pygame.draw.circle(screen, RED, (px, py), MARK_SIZE)
        #     pygame.draw.circle(screen, BLACK, (px, py), MARK_SIZE, 2)

        # # T landmarks
        # for x, y in self.landmarks_T:
        #     px = offset_x + int(x * scale)
        #     py = offset_y + int(y * scale)
        #     pygame.draw.rect(
        #         screen,
        #         BLUE,
        #         pygame.Rect(px - 5, py - 5, 10, 10),
        #     )
        #     pygame.draw.rect(
        #         screen,
        #         BLACK,
        #         pygame.Rect(px - 5, py - 5, 10, 10),
        #         2,
        #     )

        # # X landmarks
        # for x, y in self.landmarks_X:
        #     px = offset_x + int(x * scale)
        #     py = offset_y + int(y * scale)
        #     pygame.draw.line(
        #         screen, YELLOW, (px - 6, py - 6), (px + 6, py + 6), 5
        #     )
        #     pygame.draw.line(
        #         screen, YELLOW, (px - 6, py + 6), (px + 6, py - 6), 5
        #     )

        # Goalposts
        for x, y in self.landmarks_goalpost:
            px = offset_x + int(x * scale)
            py = offset_y + int(y * scale)
            pygame.draw.circle(screen, WHITE, (px, py), 10)
            pygame.draw.circle(screen, BLACK, (px, py), 10, 3)
