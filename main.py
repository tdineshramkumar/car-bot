from gui import Simulation as GuiSimulation
from models import Models
import pygame

""" Load the car image and the map image """
car_image = pygame.transform.rotate(pygame.image.load('car.png'), -90)
map_image = pygame.image.load('map.png')
""" Create a GUI simulation """
gui = GuiSimulation(map_image, car_image)
""" Setup the  common simulation parameters """
gui.__setup_simulation_parameters__(x=100, y=60, speed=40, max_distance=100)
"""Add simulations using specified model """
gui.add_simulation(Models.linear_model((5, 5, 0, -5, -5, 0)))
gui.add_simulation(Models.linear_model((10, 10, 0, -10, -10, 0)))
gui.add_simulation(Models.linear_model((1, 5, 0, -5, -1, 0)))
gui.add_simulation(Models.linear_model((0, 10, 0, -10, -0, 0)))
gui.add_simulation(Models.linear_model((5, 6, 0, -6, -5, 0)))
gui.add_simulation(Models.linear_model((8, 8, 0, -8, -8, 0)))
gui.add_simulation(Models.linear_model((-1, 5, 0, -5, 1, 0)))
gui.add_simulation(Models.linear_model((5, 10, 0, -5, -10, 0)))
""" Run the simulation """
gui.run(0.3, 0)
