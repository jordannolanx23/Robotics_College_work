import robot
from math import pi
import random


def warmup():
    #1
    bot = robot.Robot()
    bot.set(30,50, pi/2)
    print("position: " + str(bot.to_string()))

    #5 and #6
    bot.set_noise(5.0, 0.1, 5.0)

    #2
    bot = bot.move(-pi/2 , 15)
    print("sense: " + str(bot.sense()))

    #3 and #4
    bot = bot.move(-pi/2 , 10)
    print("sense: " + str(bot.sense()))

def single_move():
    p = []
    w = []
    max = 0
    index = 0
    bot = robot.Robot()

    # for loop creates and moves all particles
    for x in range(1000):
        # create robots
        temp = robot.Robot()
        p.append(temp)
        print(p[x].to_string())
        # set noise and print robots
        p[x].set_noise(0.05, 0.05, 5.0)

    # move particles
    for x in range(1000):
        p[x] = p[x].move(0.785, 5)

    #sense
    for x in range(1000):
        w.append(p[x].measurement_prob(bot.sense()))


    r = robot.resample(p, w)
    robot.print_particles(r)
    for i in range(1000):
        weight = robot.eval_set(r[i], p)
        if i == 0:
            max = weight
        elif weight > max:
            max = weight
            index = i

    bot = r[index]
    print("Most likely particle: " + str(r[index].to_string()))
    print("Average error: " +str(robot.eval_set(bot, r)))

def multi_move(turn, forward):
    p = []
    #w = []
    max = 0
    index = 0
    bot = robot.Robot()

    #initalize p
    for x in range(1000):
        # create robots
        temp = robot.Robot()
        p.append(temp)
        # print(p[x].to_string())
        # set noise and print robots
        p[x].set_noise(0.05, 0.05, 5.0)

    for i in range(len(turn)):
        w = []
        # for loop creates and moves all particles

        # move particles
        for x in range(1000):
            p[x] = p[x].move(turn[i], forward[i])

        # sense
        for x in range(1000):
            w.append(p[x].measurement_prob(bot.sense()))

        p = robot.resample(p, w)
        #robot.print_particles(r)
        for i in range(1000):
            weight = w[i]
            if i == 0:
                max = weight
            elif weight > max:
                max = weight
                index = i

        bot = p[index]
        print("Most likely particle: " + str(p[index].to_string()))
        print("Average error: " + str(robot.eval_set(bot, p)))

def main():
    #warmup()
    #single_move()
    multi_move([0.1,0.0,0.0,0.3,0.5],[5.0,5.0,2.5,3.0,5.0])

main()
