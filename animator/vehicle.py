from math import *
import numpy as np

## Class that represents a vehicle.
## Author: Bryan McKenney (based on Java code written by Lucas Guerrette)


class Vehicle:
    def __init__(self, dir, vel, pos):
        self.dir = dir # Direction vector
        self.vel = vel # Velocity vector
        self.pos = pos # Position vector
        self.turn = 0 # Control for direction of turn (0 is straight)
        self.acceleration = 0 # Control for acceleration (0 is not accelerating)
        self.FRICTION = 0.99 # Constant force of friction applied to the velocity values each timestep


    def setControls(self, turn, acceleration):
        self.turn = turn
        self.acceleration = acceleration


    def updateState(self):
        self.updateDirection(self.turn)
        self.updateVelocity(self.acceleration)
        self.updatePosition(self.vel)


    def updateDirection(self, turn):
        turnMatrix = np.array([[cos(turn), -sin(turn)], [sin(turn), cos(turn)]])
        self.dir = np.dot(self.dir, turnMatrix)


    def updateVelocity(self, acceleration):
        dirCopy = np.dot(self.dir, acceleration)
        self.vel = np.add(self.vel, dirCopy)
        self.vel = np.dot(self.vel, self.FRICTION)


    def updatePosition(self, velocity):
        self.pos = np.add(self.pos, velocity)


    def __str__(self):
        s = "Direction: " + str(self.dir[0]) + ", " + str(self.dir[1]) + "\n"
        s += "Velocity: " + str(self.vel[0]) + ", " + str(self.vel[1]) + "\n"
        s += "Position: " + str(self.pos[0]) + ", " + str(self.pos[1])
        return s
