import numpy as np
import pygame
from pygame.locals import *
import os
import sys


class Boid:
    def __init__(self):
        pass


    def set_position(self, x, y):
        pass



if __name__=='__main__':
    os.environ['SDL_VIDEO_WINDOW_POS'] = '2,30'
    pygame.init()
    screen = pygame.display.set_mode((512,512))
    screen.fill((51,51,51))
    pygame.display.set_caption('Boid Simulation')

    clock = pygame.time.Clock()