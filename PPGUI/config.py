import pygame
from constants import *

"""
Tello Axes (drone facing up)
       +x
        |
 +y <---|---> -y
        |
       -x

       180
        |
270 <---|---> 90
        |
        0
"""

"""
Map Axes (TBC - Clockwise is negative; Checked on PPGUI)
       +y
        |
 -x <---|---> -+x
        |
       -y

        0
        |
 90 <---|---> 270
        |
        180
"""

class Background(pygame.sprite.Sprite):
    def __init__(self, image_path, location):
        pygame.sprite.Sprite.__init__(self)
        try:
            # Load original image
            original_image = pygame.image.load(image_path)
            # Scale image to our field dimensions while maintaining aspect ratio
            self.image = pygame.transform.scale(original_image, (SCREEN_WIDTH, SCREEN_HEIGHT))
            self.rect = self.image.get_rect()
            self.rect.left, self.rect.top = location
            print("Background image loaded and scaled successfully")
        except Exception as e:
            print(f"Error loading background image: {e}")
            self.image = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
            self.image.fill(WHITE)
            self.rect = self.image.get_rect()
            self.rect.left, self.rect.top = location