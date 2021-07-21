import numpy as np
import pygame
from pygame.locals import *
import os
import sys

SCREEN_SIZE = WIDTH, HEIGHT = (1024, 1024)
SCREEN_CENTER = (WIDTH//2, HEIGHT//2)
SCREEN_BG_COLOR = (40,40,40)
NUM_BOIDS = 10

class Boid(pygame.sprite.Sprite):
    def __init__(self, simulator):
        self.groups = [simulator.boid_sprites]
        pygame.sprite.Sprite.__init__(self, self.groups)
        
        self.color = (240,240,240)
        self.simulator = simulator
        self.image = pygame.Surface((32,32))
        self.image.fill(self.color)
        self.acceleration = pygame.Vector2(0.5,0.0)
        self.velocity = pygame.Vector2(5.0,0.0)
        self.position = pygame.Vector2(SCREEN_CENTER)
        self.rect = self.image.get_rect()

    def update(self):

        self.rect.centerx = int(self.position[0])
        self.rect.centery = int(self.position[1])

    def update_kinematics(self,dt):
        self.velocity += self.acceleration*dt
        self.position += self.velocity*dt




class BoidSimulator:
    def __init__(self):
        self.boid_sprites = pygame.sprite.Group()
        self.boids = []
        for _ in range(NUM_BOIDS):
            self.boids.append(Boid(self))

        dist = 50
        deltas = [(0,0), (dist,-dist), (-dist,-dist), (-dist,dist), (dist,dist)]
        positions = [pygame.Vector2(SCREEN_CENTER) + delta for delta in deltas]
        print(positions)
        for i, position in enumerate(positions):
            self.boids[i].position = pygame.Vector2(position)



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

            self.boid_sprites.update()
            self.boid_sprites.draw(screen)

            for boid in self.boid_sprites:
                boid.update_kinematics(dt)
                # print(f'dt-{dt}, boid_position-{boid.position[0]},{boid.position[1]}, rect-{boid.rect.centerx},{boid.rect.centery}')

            pygame.display.flip()

        pygame.quit()
            

if __name__=='__main__':
    boid_simulator = BoidSimulator()

    boid_simulator.run()