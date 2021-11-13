from math import *
import random

# A set of 4 landmarks in the environment
# Landmarks are the parts of the environment that a robot will sense its
# distance from
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0

class Robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;

    def set(self, new_x, new_y, new_orientation):
        ''' Set a new x,y, and orientation for the robot

            Args:
                new_x: x coordinate
                new_y: y coordinate
                new_orientation: theta '''

        if new_x < 0 or new_x >= world_size:
            raise ValueError, 'X coordinate out of bound'
        if new_y < 0 or new_y >= world_size:
            raise ValueError, 'Y coordinate out of bound'
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)


    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        ''' Set the noise/uncertainty for the robot

        Args:
            new_f_noise: sigma value for driving forward
            new_t_noise: sigma value for turning in place
            new_s_noise: sigma value for sensor measurements
        '''
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);


    def sense(self):
        ''' Get a sensor measurement

        Return:
            Z: array of distance values from each landmark
        '''
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z


    def move(self, turn, forward):
        ''' Drive the robot

        Args:
            turn: radian value to turn in place before driving forward
            forward: meter distance to drive forward
        Return:
            res: new robot instance after applying the movement
        '''
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'

        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size    # cyclic truncate
        y %= world_size

        # set particle
        res = Robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res

    def Gaussian(self, mu, sigma, x):
        ''' Calculates the probability of x for 1-dim Gaussian with mean mu and
        sigma

        Args:
            mu: mean value
            sigma: uncertainty value
            x: data point
        Return:
            probability value of x
        '''
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi *
                (sigma ** 2))



    def measurement_prob(self, measurement):
        ''' Calculates how likely a measurement should be

        Args:
            measurement: sensor measurement

        Return:
            prob: probability that the measurement is correct
        '''

        prob = 1.0;
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob



    def to_string(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


def eval_set(r, p):
    sum = 0.0;
    for i in range(len(p)): # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))



def resample(p, w):
    # Re-sampling
    # Sample particles from p with probability proportional to w
    # p3 particles are drawn from p with probability w
    p3 = []
    N = len(p)
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    return p3


def print_particles(p):
    ps = []
    for i in range(len(p)):
        config = [p[i].x, p[i].y, p[i].orientation]
        ps.append(config)
    print ps

