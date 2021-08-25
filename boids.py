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
NUM_BOIDS = 3
FPS = 30


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

        # shape
        self.spawn_pad = 50
        self.position = pygame.Vector2(randint(self.spawn_pad, WIDTH-self.spawn_pad), randint(self.spawn_pad, HEIGHT-self.spawn_pad))

        # kinematics
        self.acceleration = pygame.Vector2(0.0,0.0)
        self.velocity = pygame.Vector2(randint(-5,5),randint(-5,5)) * 20
        self.angle = atan2(self.velocity[1], self.velocity[0])

        # perception
        self.perception = randint(20,30)

        self.max_speed = randint(50,100)
        self.max_force = randint(5,8)


    def wrap_horizontally(self, x):
        # if x <= 0:
        #     x = WIDTH
        # elif x >= WIDTH:
        #     x = 0
        
        return x%WIDTH


    def wrap_vertically(self, y):
        # if y <= 0:
        #     y = HEIGHT
        # elif y >= HEIGHT:
        #     y = 0
        return y%HEIGHT


    def update(self):
        self.position[0] = self.wrap_horizontally(int(self.position[0]))
        self.position[1] = self.wrap_vertically(int(self.position[1]))
        self.rect.centerx = self.position[0]
        self.rect.centery = self.position[1]


    def update_kinematics(self,dt):
        # self.update_acceleration()
        # self.acceleration = pygame.Vector2(random()*4 - 2, random()*4 - 2)
        self.velocity += self.acceleration*dt
        self.position += self.velocity*dt
        self.angle = atan2(self.velocity[1], self.velocity[0])

    
    def draw_boid(self, screen, position=None, angle=None):
        position = self.position if position is None else position
        angle = self.angle if angle is None else angle
        
        points = self.shape_points

        rotated_points = points @ np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]]).T 

        points = rotated_points + position
        self.rect = pygame.draw.polygon(screen, self.color, (tuple(points[0].flatten()), tuple(points[1].flatten()), tuple(points[2].flatten()), tuple(points[3].flatten())))




class BoidSimulator:
    def __init__(self):
        # create boids and assign let them assign themselves to a group
        self.boid_sprites = pygame.sprite.Group()
        self.boids = []
        for _ in range(NUM_BOIDS):
            self.boids.append(Boid(self))



    def run(self):
        os.environ['SDL_VIDEO_WINDOW_POS'] = '2,30'
        pygame.init()
        pygame.display.set_caption('Boid Simulation')
        screen = pygame.display.set_mode(SCREEN_SIZE, flags=pygame.DOUBLEBUF)
        screen.fill(SCREEN_BG_COLOR)

        clock = pygame.time.Clock()

        sim_running = True
        
        while sim_running:
            # update clock
            dt = clock.tick(FPS) / 1000

            # re-draw background
            screen.fill(SCREEN_BG_COLOR)

            # handle events (only quit for now)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sim_running = False
                    pygame.quit()

            # draw 
            for sprite in self.boid_sprites:
                sprite.draw_boid(screen)

            # update acceleration
            for boid in self.boid_sprites:
                avg_vel = np.zeros((1,2))
                com = np.zeros((1,2))
                alignment = np.zeros((1,2))
                cohesion = np.zeros((1,2))
                separation = np.zeros((1,2))
                avg_v = np.zeros((1,2))
                avg_wall = np.zeros((1,2))
                count = 0
                # boid alignment
                for sprite in self.boid_sprites:
                    dist = np.linalg.norm(sprite.position-boid.position)
                    if sprite is not boid and dist < boid.perception:
                        avg_vel += sprite.velocity
                        count += 1
                if count > 0:
                    avg_vel /= count
                    if np.linalg.norm(avg_vel) > 0:
                        avg_vel = (avg_vel/np.linalg.norm(avg_vel)) * boid.max_speed
                    alignment = avg_vel - boid.velocity
                    if np.linalg.norm(alignment) > 0:
                        alignment = (alignment/np.linalg.norm(alignment)) * boid.max_force

                count = 0
                # boid cohesion
                for sprite in self.boid_sprites:
                    dist = np.linalg.norm(sprite.position-boid.position)
                    if sprite is not boid and dist < boid.perception:
                        com += sprite.position
                        count += 1
                if count > 0:
                    com /= count
                    vec_com = com - boid.position
                    if np.linalg.norm(vec_com) > 0:
                        vec_com = (vec_com/np.linalg.norm(vec_com)) * boid.max_speed
                    cohesion = vec_com - boid.velocity 
                    if np.linalg.norm(cohesion) > 0:
                        cohesion = (cohesion/np.linalg.norm(cohesion)) * boid.max_force

                count = 0
                # boid separation
                for sprite in self.boid_sprites:
                    dist = np.linalg.norm(sprite.position-boid.position)
                    if sprite is not boid and dist < boid.perception:
                        diff = (boid.position - sprite.position) / dist
                        avg_v += diff
                        count += 1
                if count > 0:
                    avg_v /= count
                walls = [pygame.Vector2(boid.position[0],0), 
                         pygame.Vector2(0,boid.position[1]),
                         pygame.Vector2(boid.position[0],HEIGHT),
                         pygame.Vector2(WIDTH,boid.position[1])]
                wall_count = 0
                for wall in walls:
                    dist = np.linalg.norm(wall-boid.position)
                    if dist < boid.perception*5:
                        diff = (boid.position - wall) / dist**2
                        avg_wall += diff
                        wall_count += 1
                if count > 0 or wall_count > 0:
                    avg_v += 100*avg_wall/wall_count
                    if np.linalg.norm(avg_v) > 0:
                        avg_v = (avg_v/np.linalg.norm(avg_v)) * boid.max_speed
                    separation = avg_v - boid.velocity
                    if np.linalg.norm(separation) > 0:
                        separation = (separation/np.linalg.norm(separation)) * boid.max_force

                acc_total = 1*alignment.flatten() + 1*cohesion.flatten() + 1*separation.flatten()
                
                # update boid acceleration
                boid.acceleration = pygame.Vector2(acc_total[0], acc_total[1])



                # separation = (sum([(np.linalg.norm(nbr.position - boid.position))**-2 for nbr in self.boid_sprites if nbr is not boid]) / (NUM_BOIDS-1) ) - (np.linalg.norm(boid.position))**-2
                # print(f'{alignment}, {cohesion}, {separation}, {acc_total}')

            # update kinematics
            for boid in self.boid_sprites:
                boid.update_kinematics(dt)

            # update rects
            self.boid_sprites.update()

            pygame.display.flip()

        pygame.quit()
            

if __name__=='__main__':
    boid_simulator = BoidSimulator()

    boid_simulator.run()