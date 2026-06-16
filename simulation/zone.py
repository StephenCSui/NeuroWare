import pygame
from config import LOADING_ZONE, STORING_ZONE, GOAL_ZONE, COLORS


class Zone:
    def __init__(self, name, rect, color_key, label_color):
        self.name        = name
        self.rect        = pygame.Rect(*rect)
        self.color_key   = color_key
        self.label_color = label_color

    def contains(self, x, y):
        return self.rect.collidepoint(x, y)

    def center(self):
        return self.rect.centerx, self.rect.centery

    def draw(self, surface, font):
        overlay = pygame.Surface((self.rect.width, self.rect.height), pygame.SRCALPHA)
        r, g, b = COLORS[self.color_key]
        overlay.fill((r, g, b, 50))
        surface.blit(overlay, (self.rect.x, self.rect.y))
        pygame.draw.rect(surface, COLORS[self.color_key], self.rect, 2, border_radius=6)
        label = font.render(self.name, True, self.label_color)
        surface.blit(label, (self.rect.centerx - label.get_width() // 2,
                              self.rect.centery - label.get_height() // 2))


def make_zones():
    return {
        "loading": Zone("LOADING", LOADING_ZONE, "loading", COLORS["loading"]),
        "storing": Zone("STORING", STORING_ZONE, "storing", COLORS["storing"]),
        "goal":    Zone("GOAL",    GOAL_ZONE,    "goal",    COLORS["goal"]),
    }
