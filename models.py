from pygame import Vector2
from math import degrees, radians, tan, tanh
from numpy import linspace


class Car:
    """ Models a car moving with a constant speed, which is controlled by the external agent by means
     of steering; Proceed ahead with simulation by calling update function by a given delta time;
     Angles are stored in degrees, and Vector2 class uses angles in degrees and math functions uses radians
    Positions and Velocity are unit-less and are frequently mapped directly to pixels on the screen after
    rounding to nearest integers """

    def __init__(self, x, y, length, width, speed, angle=0.0, steering_angle=0.0, max_steering_angle=50):
        """ Setup the initial values, to define the initial position and orientation, and initial speed and
        also setup the maximum steering angle which is used to update the steering using set_steering function.
        Also setup the dimensions of the car using length and width parameters.
        """
        """ The initial position of the car """
        self.position = Vector2(x, y)
        """ The initial orientation of the car (in degrees) """
        self.angle = angle
        """ The constant speed of the car used in the simulation """
        self.speed = speed
        """ The initial steering angle of the car (in degrees) """
        self.steering_angle = steering_angle
        """ the maximum steering angle is used to update the steering angle """
        self.max_steering_angle = max_steering_angle
        """ Set the dimensions of the car, length is used to calculate the turning radius, 
        width and length may be used by later modules to identify boundaries to detect collisions """
        self.length = length
        self.width = width

    def set_steering(self, steering_factor):
        """ Update the steering angle to a new value, used by the external agent,
        input is a factor which is multiplied by the max_steering_angle to obtain the new steering angle """
        assert -1 <= steering_factor <= 1
        self.steering_angle = steering_factor * self.max_steering_angle

    def update(self, dt):
        """ Update the car positions and orientations as simulation progresses by given delta time """
        """ First obtain the angular velocity resulting from steering the car, which will cause the car to rotate """
        if self.steering_angle == 0:
            angular_velocity = 0.0
        else:
            """ Calculate the turning radius of the car as a result of turning and obtain the angular velocity """
            turning_radius = self.length / tan(radians(self.steering_angle))
            angular_velocity = self.speed / turning_radius
        """ Update the position based on current velocity (using speed and angle) """
        self.position += Vector2(self.speed, 0).rotate(-self.angle) * dt
        """ Update the orientation of the car based on angular velocity """
        self.angle += degrees(angular_velocity) * dt


class Utils:
    @staticmethod
    def vector_to_point(vec_: Vector2):
        """ Converts the vector to a integer tuple """
        return int(vec_.x), int(vec_.y)

    @staticmethod
    def get_boundary_points(car: Car):
        """ Returns some of the boundary points of the car used to detect collisions """
        """ This lambda returns a point with respect to the car, with the point identified as 
        fractions of length and width """

        def __car_point__(l_fac, w_fac):
            return car.position + Vector2(car.length * l_fac, car.width * w_fac).rotate(-car.angle)
        boundary_points_ = [__car_point__(0, 0.5), __car_point__(0, -0.5), __car_point__(0.5, 0),
                            __car_point__(-0.5, 0), __car_point__(0.5, 0.5), __car_point__(0.5, -0.5),
                            __car_point__(-0.5, 0.5), __car_point__(-0.5, -0.5)]
        return list(map(Utils.vector_to_point, boundary_points_))

    @staticmethod
    def detect_collision(car: Car, layout_):
        """ Detects collision by checking if there are any obstacles at the boundaries """
        for point_ in Utils.get_boundary_points(car):
            if layout_[point_]:
                """ If there is any obstacle at the given point of the layout """
                return True
        """ If there are no obstacles at the boundary points """
        return False

    @staticmethod
    def get_distance_from_obstacle_along_direction(car: Car, layout_, angle_, max_distance_, resolution_=100,
                                                   return_vector=False):
        """ This function returns the distance from the obstacle along given direction wrt car if any obstacle
        in the given field of vision """
        vec_ = Vector2()
        for distance_ in linspace(0, max_distance_, resolution_):
            vec_.from_polar((distance_, -car.angle - angle_))
            if layout_[Utils.vector_to_point(car.position + vec_)]:
                """ If obstacle then return the distance """
                return distance_ if not return_vector else vec_
        vec_.from_polar((max_distance_, -car.angle - angle_))
        return max_distance_ if not return_vector else vec_

    @staticmethod
    def get_obstacle_distances(car: Car, layout_, max_distance_, resolution_=100):
        """ This function returns the distances to obstacles along left, front-left,
        front, front-right and right directions """
        return (
            Utils.get_distance_from_obstacle_along_direction(car, layout_, 90, max_distance_, resolution_),
            Utils.get_distance_from_obstacle_along_direction(car, layout_, 45, max_distance_, resolution_),
            Utils.get_distance_from_obstacle_along_direction(car, layout_, 0, max_distance_, resolution_),
            Utils.get_distance_from_obstacle_along_direction(car, layout_, -45, max_distance_, resolution_),
            Utils.get_distance_from_obstacle_along_direction(car, layout_, -90, max_distance_, resolution_),
        )

    @staticmethod
    def get_fractional_obstacle_distances(car: Car, layout_, max_distance_, resolution_=100):
        """ This function returns the distances as fractions (obtained by dividing by max_distance)
        to obstacles along left, front-left, front, front-right and right directions """
        return tuple(map(lambda distance: distance/max_distance_,
                         Utils.get_obstacle_distances(car, layout_, max_distance_, resolution_)))


class Simulation(Car):
    """ This is the simulation wrapper around the car class, runs the car on the given layout till collision occurs,
    obtains the steering input using the model provided. Model is expected to be a function which takes
    distances (in fraction) as input and returns steering factor"""
    def __init__(self, layout, model, x, y, length, width, speed, max_distance, resolution=1000, angle=0.0,
                 steering_angle=0.0, max_steering_angle=50):
        self.layout = layout
        self.model = model
        self.max_distance = max_distance
        self.resolution = resolution
        self.collision = False
        self.iterations = 0
        self.obstacle_distances = 0
        Car.__init__(self, x, y, length, width, speed, angle, steering_angle, max_steering_angle)
        """ Assert that no collision occurs at the initial position """
        # assert not Utils.detect_collision(self, layout)

    def update(self, dt, debug=True):
        """ Progresses the simulation by given delta time by updating the car and scores and checking for collisions"""
        if not self.collision:
            """ Call update on the car """
            distances = Utils.get_fractional_obstacle_distances(self, self.layout, self.max_distance, self.resolution)
            assert all(map(lambda distance__: 1 >= distance__ >= 0, distances))
            steering_factor = self.model(distances)
            Car.set_steering(self, steering_factor)
            Car.update(self, dt)
            self.collision = Utils.detect_collision(self, self.layout)
            """ Update the number of iterations and obstacle distances """
            self.iterations += 1
            self.obstacle_distances += sum(distances)
            if debug:
                """ While debugging output scores """
                print('Running Iteration:', self.iterations,
                      'With obstacle avoidance score:', self.obstacle_distances)
                if self.collision:
                    print('Collision occurred at iteration:', self.iterations,
                          'With obstacle avoidance score:', self.obstacle_distances)

    def check_collision(self):
            """ Determine if collision occured and end of simulation """
            return self.collision


class Models:
    @staticmethod
    def linear_model(parameters):
        assert len(parameters) == 6
        """ Construct a linear model with given set of parameters, 
        with first 5 as multipliers and last one the bias """

        def __model__(distances):
            """ Return the steering factor based on distances """
            return tanh(distances[0] * parameters[0] + distances[1] * parameters[1] + distances[2] * parameters[2] +
                        distances[3] * parameters[3] + distances[4] * parameters[4] + parameters[5])
        return __model__
