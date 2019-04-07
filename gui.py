import pygame
import numpy as np
from itertools import product
import models


class Simulation:
    """ GUI version of the simulation. This class visualizes the simulations """

    def __init__(self, map_image, car_image, obstacle_color=(0, 0, 0), fps=60):
        pygame.init()
        self.map_image = map_image
        self.car_image = car_image
        self.fps = fps
        """ Obtain the width and height of the map and the car """
        self.WIDTH, self.HEIGHT = map_image.get_width(), map_image.get_height()
        self.car_length, self.car_width = car_image.get_width(), car_image.get_height()

        """ Update the layout based on positions of obstacles """
        self.layout = np.zeros((self.WIDTH, self.HEIGHT), dtype=np.bool)
        for x, y in product(range(self.WIDTH), range(self.HEIGHT)):
            if map_image.get_at((x, y)) == obstacle_color:
                self.layout[x, y] = True

        """ Create display based on size of map """
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption('Simple Car Simulation')
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font('freesansbold.ttf', self.car_width // 2)

        self.simulations = []
        self.labels = []
        self.labels_rect = []

    def __setup_simulation_parameters__(self, x, y, speed, max_distance, resolution=1000, angle=0.0,
                                        steering_angle=0.0, max_steering_angle=50):
        """ Save the common parameters for all the simulations to run """
        self.start_x = x
        self.start_y = y
        self.speed = speed
        self.max_distance = max_distance
        self.resolution = resolution
        self.angle = angle
        self.steering_angle = steering_angle
        self.max_steering_angle = max_steering_angle

    def add_simulation(self, model):
        """ Construct a simulation using the set parameters and run them using the given model """
        simulation = models.Simulation(self.layout, model, self.start_x, self.start_y, self.car_length, self.car_width,
                                self.speed, self.max_distance, self.resolution, self.angle, self.steering_angle,
                                self.max_steering_angle)
        index = len(self.simulations)
        self.simulations.append(simulation)
        """ Construct the labels and their rects """
        self.labels.append(self.font.render(str(index), True, (0, 0, 0), (255, 255, 255)))
        self.labels_rect.append(self.labels[-1].get_rect())

    def run(self, dt, max_iterations=0):
        """ Run the simulations using given delta time and visualize them """
        if not self.simulations:
            print("No simulations to run")
            return
        current_iteration = 0
        while True:
            quit_loop = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    quit_loop = True
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_p:
                        """ If P is pressed then print stats """
                        for index, simulation in enumerate(self.simulations):
                            print("Simulation:", index, "Iterations:", simulation.iterations,
                                  "Obstacle Distances:", simulation.obstacle_distances,
                                  "Collision:", simulation.check_collision())
            if quit_loop:
                """ On quit, break loop """
                pygame.quit()
                break
            """ Draw the map """
            self.screen.blit(self.map_image, self.map_image.get_rect())
            """ Draw each of the car """
            for index, simulation in enumerate(self.simulations):
                new_car_image = pygame.transform.rotate(self.car_image, simulation.angle)
                new_car_rect = new_car_image.get_rect()
                new_car_rect.center = simulation.position
                """ Label each car """
                self.screen.blit(new_car_image, new_car_rect)
                self.labels_rect[index].center = simulation.position
                self.screen.blit(self.labels[index], self.labels_rect[index])
                if not simulation.check_collision():
                    """ Run till collision occurs """
                    simulation.update(dt, debug=False)
                    if simulation.check_collision():
                        """ If collision then log """
                        print("Simulation:", index, "Completed. Score:", simulation.iterations,
                              " Avoidance:", simulation.obstacle_distances)
                        print("Running Simulations:",
                              [index__ for index__, simulation_ in enumerate(self.simulations) if
                               not simulation_.check_collision()])
            if max_iterations > 0:
                """ this is used if GUI must be terminated after some amount of iterations"""
                current_iteration += 1
                if current_iteration > max_iterations:
                    break
            pygame.display.update()
            self.clock.tick(self.fps)
