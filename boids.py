import numpy as np
from math import cos, sin , tan, atan, atan2, pi, tau, radians, degrees
import pygame
from pygame.locals import *
import os
import sys
from config import *
from random import choice, random, randint
import numpy as np



class Boid(pygame.sprite.Sprite):
    def __init__(self, simulator):
        self.simulator = simulator

        # assign self to group
        self.groups = [self.simulator.boid_sprites]
        pygame.sprite.Sprite.__init__(self, self.groups)

        # set shape points
        self.shape_scale_factor = 0.9
        self.shape_points = np.array([[0,0],
                                      [-5, -26/3],
                                      [10, 0],
                                      [-5, 26/3]]) 
        self.shape_points *= self.shape_scale_factor
        
        # appearance
        self.color = choice(BOID_COLORS)

        # spawn
        self.spawn_pad = 100
        self.position = pygame.Vector2(randint(self.spawn_pad, WIDTH-self.spawn_pad), randint(self.spawn_pad, HEIGHT-self.spawn_pad))

        # kinematics
        self.acceleration = pygame.Vector2(0.0, 0.0)
        self.velocity = pygame.Vector2(randint(-5,5), randint(-5,5)) * 30
        self.angle = atan2(self.velocity[1], self.velocity[0])

        # perception
        self.max_range = randint(1200,1250)
        self.max_vision = radians(randint(125,165))

        self.max_speed = randint(200,250)
        self.max_acceleration = randint(7,10)


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
        if np.linalg.norm(self.acceleration) > 0:
            self.acceleration = (self.acceleration / np.linalg.norm(self.acceleration)) * self.max_acceleration


        self.velocity += self.acceleration*dt

        self.position += self.velocity*dt
        self.angle = atan2(self.velocity[1], self.velocity[0])

    
    def draw_boid(self, screen):
        position = self.position 
        angle = atan2(self.velocity[1], self.velocity[0]) 
        
        points = self.shape_points

        rotated_points = points @ np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]]).T 

        points = rotated_points + position
        self.rect = pygame.draw.polygon(screen, self.color, (tuple(points[0].flatten()), tuple(points[1].flatten()), tuple(points[2].flatten()), tuple(points[3].flatten())))


    def normalize_angle(self, angle):
        return atan2(sin(angle), cos(angle))

    def draw_perception_arc(self, screen):
        position = self.position
        angle = atan2(self.velocity[1], self.velocity[0])
        # ahead = np.array([[int(self.max_range), 0]])
        # ahead = ahead @ np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]]).T 
        # ahead = (position + ahead).flatten()
        # pygame.draw.line(screen, PERCEPTION_COLOR, tuple(position), tuple(ahead), 1)
        
        # note angle was computed using image topleft coords therefore clockwise is positive
        # pygame draw arc takes in angles in the usual anti-clockwise positive convention with +/- pi range
        start_angle = self.normalize_angle(-angle - self.max_vision)
        end_angle = self.normalize_angle(-angle + self.max_vision)
        size = int(self.max_range)
        left = int(position[0] - size)
        top = int(position[1] - size)
        rect = pygame.rect.Rect(left, top, size*2, size*2)
        rect.center = self.position
        rect2 = pygame.rect.Rect(left+size//2, top+size//2, size, size)
        rect2.center = self.position

        pygame.draw.arc(screen, PERCEPTION_COLOR, rect, start_angle, end_angle, 1)
        # pygame.draw.arc(screen, PERCEPTION_COLOR, rect2, start_angle, end_angle, 3)



class BoidSimulator:
    def __init__(self):
        os.environ['SDL_VIDEO_WINDOW_POS'] = '2,30'
        pygame.init()
        pygame.display.set_caption('Boid Simulation')
        self.screen = pygame.display.set_mode(SCREEN_SIZE, flags=pygame.DOUBLEBUF)
        self.screen.fill(SCREEN_BG_COLOR)

        self.clock = pygame.time.Clock()

        self.sim_running = True
        # create boids and assign let them assign themselves to a group
        self.boid_sprites = pygame.sprite.Group()
        self.boids = []
        for _ in range(NUM_BOIDS):
            self.boids.append(Boid(self))

        self.boids[-1].color = THE_ONE_COLOR
        # draw 
        for sprite in self.boid_sprites:
            sprite.draw_boid(self.screen)
        sprite.draw_perception_arc(self.screen)

    def compute_separation(self, this_boid):
        visible_boids = [some_boid for some_boid in self.boid_sprites if some_boid is not this_boid and this_boid.can_see(some_boid)]

        avg_away = pygame.Vector2(0.0, 0.0)
        separation = pygame.Vector2(0.0, 0.0)
        for other_boid in visible_boids:
            rel_dist = np.linalg.norm(other_boid.position - this_boid.position)

            # scaled away vector by inverse distance
            away = (this_boid.position - other_boid.position) / (rel_dist**2 + 1e-16)
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
        
        
        while self.sim_running:
            # update clock
            dt = self.clock.tick(FPS) / 1000

            # re-draw background
            self.screen.fill(SCREEN_BG_COLOR)

            # handle events (only quit for now)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.sim_running = False
                    pygame.quit()

            # update acceleration
            for boid in self.boid_sprites:
                separation = self.compute_separation(boid)
                alignment = self.compute_alignment(boid)
                cohesion = self.compute_cohesion(boid)
                containment = self.compute_containment(boid)

                # print(f'alignment - {alignment}, cohesion - {cohesion}, separation = {separation}, containment - {containment}')
                acc_total = 0.3*boid.acceleration +  2*alignment + 1.8*cohesion + 1.5*separation + 1*containment
                
                # update boid acceleration
                boid.acceleration = pygame.Vector2(acc_total[0], acc_total[1])

            # update kinematics
            for boid in self.boid_sprites:
                boid.update_kinematics(dt)

            # update rects
            self.boid_sprites.update()

            # draw 
            for sprite in self.boid_sprites:
                sprite.draw_boid(self.screen)
            sprite.draw_perception_arc(self.screen)

            pygame.display.flip()

        pygame.quit()
            

if __name__=='__main__':
    boid_simulator = BoidSimulator()

    boid_simulator.run()