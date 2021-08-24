import numpy as np
from math import cos, sin , tan, atan, atan2, pi, tau, radians, degrees
import pygame
from pygame.locals import *
import os
import sys
from config import *
from random import choice, random, randint
import numpy as np

SCREEN_SIZE = WIDTH, HEIGHT = (1024, 1024)
SCREEN_CENTER = (WIDTH//2, HEIGHT//2)
SCREEN_BG_COLOR = (40,40,40)
NUM_BOIDS = 10

class Boid(pygame.sprite.Sprite):
    def __init__(self, simulator):
        self.simulator = simulator

        # assign self ot group
        self.groups = [self.simulator.boid_sprites]
        pygame.sprite.Sprite.__init__(self, self.groups)

        # set shape points
        self.shape_points = np.array([[0,0],
                                      [-5, -26/3],
                                      [10, 0],
                                      [-5, 26/3]])
        
        # appearance
        self.color = choice(multi_colors)
        self.image = pygame.Surface((15,15))
        self.rect = pygame.draw.polygon(self.image, self.color, ((0, 5), (0, 10), (10, 10), (10, 15), (15, 8), (10, 0), (10, 5)))
        # self.image.fill(self.color)
        self.spawn_pad = 50
        self.position = pygame.Vector2(randint(self.spawn_pad, WIDTH-self.spawn_pad), randint(self.spawn_pad, HEIGHT-self.spawn_pad))
        self.acceleration = pygame.Vector2(0.0,0.0)
        self.velocity = pygame.Vector2(randint(-5,5),randint(-5,5)) * 10
        self.angle = atan2(self.velocity[1], self.velocity[0])
        # self.rect = self.image.get_rect()

    def update(self):

        self.rect.centerx = int(self.position[0])
        self.rect.centery = int(self.position[1])

    def update_kinematics(self,dt):
        self.acceleration = pygame.Vector2(random()*4 - 2, random()*4 - 2)
        self.velocity += self.acceleration*dt
        self.position += self.velocity*dt

    
    def draw_boid(self, screen, position=None, angle=None):
        position = self.position if position is None else position
        angle = self.angle if angle is None else angle
        
        points = self.shape_points

        rotated_points = points @ np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]]).T 

        points = rotated_points + position
        self.rect = pygame.draw.polygon(screen, self.color, (tuple(points[0].flatten()), tuple(points[1].flatten()), tuple(points[2].flatten()), tuple(points[3].flatten())))









class BoidSimulator:
    def __init__(self):
        self.boid_sprites = pygame.sprite.Group()
        self.boids = []
        for _ in range(NUM_BOIDS):
            self.boids.append(Boid(self))

        # dist = 50
        # deltas = [(0,0), 
        #           (dist,-dist), (-dist,-dist), (-dist,dist), (dist,dist),
        #           (2*dist,-2*dist), (-2*dist,-2*dist), (-2*dist,2*dist), (2*dist,2*dist), (3*dist, 0)]
        # positions = [pygame.Vector2(SCREEN_CENTER) + delta for delta in deltas]
        # for i, position in enumerate(positions):
        #     self.boids[i].position = pygame.Vector2(position)



    def run(self):
        os.environ['SDL_VIDEO_WINDOW_POS'] = '2,30'
        pygame.init()
        pygame.display.set_caption('Boid Simulation')
        screen = pygame.display.set_mode(SCREEN_SIZE, flags=pygame.DOUBLEBUF)
        screen.fill(SCREEN_BG_COLOR)

        clock = pygame.time.Clock()

        sim_running = True
        FPS = 30
        while sim_running:

            dt = clock.tick(FPS) / 1000
            screen.fill(SCREEN_BG_COLOR)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sim_running = False
                    pygame.quit()

            for sprite in self.boid_sprites:
                sprite.draw_boid(screen)
            self.boid_sprites.update()

            for boid in self.boid_sprites:
                boid.update_kinematics(dt)
                # print(f'dt-{dt}, boid_position-{boid.position[0]},{boid.position[1]}, rect-{boid.rect.centerx},{boid.rect.centery}')

            pygame.display.flip()

        pygame.quit()
            

if __name__=='__main__':
    boid_simulator = BoidSimulator()

    boid_simulator.run()