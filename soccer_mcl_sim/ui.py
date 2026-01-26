# ui.py
import pygame


class InputField:
    def __init__(self, x, y, w, h, label, value=0.0, min_val=-999, max_val=999):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label
        self.text = str(value)
        self.active = False
        self.min_val = min_val
        self.max_val = max_val

    def get_value(self):
        try:
            v = float(self.text)
            return max(self.min_val, min(self.max_val, v))
        except:
            return 0.0

    def set_value(self, v):
        v = max(self.min_val, min(self.max_val, v))
        self.value = v
        self.text = f"{v:.1f}"

    def draw(self, screen, font):
        color = (255, 255, 255) if self.active else (220, 220, 220)
        pygame.draw.rect(screen, color, self.rect, border_radius=6)
        pygame.draw.rect(screen, (50, 50, 50), self.rect, 2, border_radius=6)

        label_surf = font.render(self.label, True, (0, 0, 0))
        screen.blit(label_surf, (self.rect.x, self.rect.y - 18))

        txt = font.render(self.text, True, (0, 0, 0))
        screen.blit(txt, (self.rect.x + 5, self.rect.y + 5))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.active = self.rect.collidepoint(event.pos)

        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            elif event.unicode in "0123456789.-":
                self.text += event.unicode



class Button:
    def __init__(self, x, y, w, h, text, callback):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.callback = callback

    def draw(self, screen, font):
        pygame.draw.rect(screen, (230, 230, 230), self.rect)
        pygame.draw.rect(screen, (20, 20, 20), self.rect, 2)
        txt = font.render(self.text, True, (0, 0, 0))
        screen.blit(
            txt,
            txt.get_rect(center=self.rect.center),
        )

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.callback()


class Toggle:
    def __init__(self, x, y, label, value=False):
        self.rect = pygame.Rect(x, y, 20, 20)
        self.label = label
        self.value = value

    def draw(self, screen, font):
        pygame.draw.rect(screen, (255, 255, 255), self.rect)
        pygame.draw.rect(screen, (0, 0, 0), self.rect, 2)
        if self.value:
            pygame.draw.line(
                screen, (0, 0, 0),
                self.rect.topleft,
                self.rect.bottomright, 3
            )
            pygame.draw.line(
                screen, (0, 0, 0),
                self.rect.topright,
                self.rect.bottomleft, 3
            )

        txt = font.render(self.label, True, (0, 0, 0))
        screen.blit(txt, (self.rect.right + 8, self.rect.y))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.value = not self.value
