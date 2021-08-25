import numpy as np
from math import cos, sin , tan, atan, atan2, pi, tau, radians, degrees
import pygame
from pygame.locals import *
import os
import sys
from config import *
from random import choice, random, randint
import numpy as np

SCREEN_SIZE = WIDTH, HEIGHT = (800, 600)
SCREEN_CENTER = (WIDTH//2, HEIGHT//2)
SCREEN_BG_COLOR = (40,40,40)
NUM_BOIDS = 21
FPS = 30


class Boid(pygame.sprite.Sprite):
    def __init__(self, simulator):
        self.simulator = simulator

        # assign self ot group
        self.groups = [self.simulator.boid_sprites]
        pygame.sprite.Sprite.__init__(self, self.groups)

        # set shape points
        self.shape_scale_factor = 0.7
        self.shape_points = np.array([[0,0],
                                      [-5, -26/3],
                                      [10, 0],
                                      [-5, 26/3]]) 
        self.shape_points *= self.shape_scale_factor
        
        # appearance
        self.color = choice(greens)

        # shape
        self.spawn_pad = 100
        self.position = pygame.Vector2(randint(self.spawn_pad, WIDTH-self.spawn_pad), randint(self.spawn_pad, HEIGHT-self.spawn_pad))

        # kinematics
        self.acceleration = pygame.Vector2(0.0,0.0)
        self.velocity = pygame.Vector2(randint(-5,5),randint(-5,5)) * 30
        self.angle = atan2(self.velocity[1], self.velocity[0])

        # perception
        self.max_range = randint(200,300)
        self.max_vision = radians(randint(120,160))

        self.max_speed = randint(50,100)
        self.max_acceleration = randint(5,7)


    def wrap_horizontally(self, x):
        if x <= 0:
            x = WIDTH
        elif x >= WIDTH:
            x = 0
        
        return x


    def wrap_vertically(self, y):
        if y <= 0:
            y = HEIGHT
        elif y >= HEIGHT:
            y = 0
        return y


    def update(self):
        self.position[0] = self.wrap_horizontally(int(self.position[0]))
        self.position[1] = self.wrap_vertically(int(self.position[1]))
        self.rect.centerx = self.position[0]
        self.rect.centery = self.position[1]

    def can_see(self, sprite):
        rel_pos = sprite.position - self.position
        rel_dist = np.linalg.norm(rel_pos)
        rel_angle = atan2(rel_pos[1], rel_pos[0]) - self.angle
        if rel_dist <= self.max_range and abs(rel_angle) <= self.max_vision:
            can_see = True
        else:
            can_see = False

        return can_see

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

    def compute_separation(self, this_boid):
        visible_boids = [some_boid for some_boid in self.boid_sprites if some_boid is not this_boid and this_boid.can_see(some_boid)]

        avg_away = pygame.Vector2(0.0, 0.0)
        separation = pygame.Vector2(0.0, 0.0)
        for other_boid in visible_boids:
            rel_dist = np.linalg.norm(other_boid.position - this_boid.position)

            # scaled away vector by inverse distance
            away = (this_boid.position - other_boid.position) / (rel_dist + 1e-16)
            avg_away += away

        avg_away = avg_away / len(visible_boids) if len(visible_boids) > 0 else avg_away

        # normalize the average away vector, to be treated as velocity with max speed
        avg_away_norm = np.linalg.norm(avg_away)
        if avg_away_norm > 0:
            avg_away = (avg_away / avg_away_norm) * this_boid.max_speed

        # compute acceleration, normalize to max_force
        separation = avg_away - this_boid.velocity
        separation_norm = np.linalg.norm(separation)
        if separation_norm > 0:
            separation = (separation / separation_norm) * this_boid.max_acceleration

        return separation


    def compute_alignment(self, this_boid):
        visible_boids = [some_boid for some_boid in self.boid_sprites if some_boid is not this_boid and this_boid.can_see(some_boid)]

        avg_vel = pygame.Vector2(0.0, 0.0)
        for other_boid in visible_boids:
            avg_vel += other_boid.velocity

        avg_vel = avg_vel / len(visible_boids) if len(visible_boids) > 0 else avg_vel

        # normalize to max speed
        avg_vel_norm = np.linalg.norm(avg_vel)
        if avg_vel_norm > 0:
            avg_vel = (avg_vel / avg_vel_norm) * this_boid.max_speed

        # compute acceleration 
        alignment = avg_vel - this_boid.velocity
        alignment_norm = np.linalg.norm(alignment)
        if alignment_norm > 0:
            alignment = (alignment / alignment_norm) * this_boid.max_acceleration

        return alignment


    def compute_cohesion(self, this_boid):
        visible_boids = [some_boid for some_boid in self.boid_sprites if some_boid is not this_boid and this_boid.can_see(some_boid)]

        centroid = pygame.Vector2(0.0, 0.0)
        for other_boid in visible_boids:
            centroid += other_boid.position

        centroid = centroid / len(visible_boids) if len(visible_boids) > 0 else centroid

        toward = centroid - this_boid.position

        # normalize to max speed
        toward_norm = np.linalg.norm(toward)
        if toward_norm > 0:
            toward = (toward / toward_norm) * this_boid.max_speed

        # compute acceleration, normalize
        cohesion = toward - this_boid.velocity
        cohesion_norm = np.linalg.norm(cohesion)
        if cohesion_norm > 0:
            cohesion = (cohesion / cohesion_norm) * this_boid.max_acceleration

        return cohesion


    def compute_containment(self, this_boid):
        wall_points = [pygame.Vector2(this_boid.position[0], 0), 
                       pygame.Vector2(0, this_boid.position[1]),
                       pygame.Vector2(this_boid.position[0], HEIGHT),
                       pygame.Vector2(WIDTH, this_boid.position[1])]

        avoid = pygame.Vector2(0.0, 0.0)
        for point in wall_points:
            rel_pos = point - this_boid.position
            rel_angle = atan2(rel_pos[1], rel_pos[0]) - this_boid.angle
            rel_dist = np.linalg.norm(rel_pos)
            x, y = tuple(this_boid.velocity.elementwise()*(-1, 1))
            avoid += pygame.Vector2(y, x) / (rel_dist + 1e-16)

        avoid /= len(wall_points)

        # normalize to max speed
        avoid_norm = np.linalg.norm(avoid)
        if avoid_norm > 0:
            avoid = (avoid / avoid_norm) * this_boid.max_speed

        # compute acceleration, normalize
        containment = avoid - this_boid.velocity
        containment_norm = np.linalg.norm(containment)
        if containment_norm > 0:
            containment = (containment / containment_norm) * this_boid.max_acceleration

        return containment


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
                separation = self.compute_separation(boid)
                alignment = self.compute_alignment(boid)
                cohesion = self.compute_cohesion(boid)
                containment = self.compute_containment(boid)

                # print(f'alignment - {alignment}, cohesion - {cohesion}, separation = {separation}, containment - {containment}')
                acc_total = 1*alignment + 1*cohesion + 0.5*separation + 1.0*containment
                
                # update boid acceleration
                boid.acceleration = pygame.Vector2(acc_total[0], acc_total[1])

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