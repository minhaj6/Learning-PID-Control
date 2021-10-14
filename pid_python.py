import numpy as np
import matplotlib.pyplot as plt
import turtle
import time

# Global Params
TIMER = 0
TIME_STEP = 0.005
SETPOINT = 10
SIM_TIME = 1000        # in seconds
INITIAL_X = 0
INITIAL_Y = -100
MASS = 1 #kg
MAX_TRUST = 15 # In newtons
g = -9.81   # trust positive
V_i = 0     # Initial velocity  
Y_i = 0     # Initial height of the rocket

# PID GAINS -------------
KP = 1.0
KD = 1.0
KI = 1.0

class Simulation():
    def __init__(self) -> None:
        self.Insight = Rocket()
        self.pid = PID(KP, KI, KD, SETPOINT)
        self.screen = turtle.Screen()
        self.screen.setup(1280, 900)
        self.marker = turtle.Turtle()
        self.marker.penup()
        self.marker.left(180)
        self.marker.goto(15, SETPOINT)
        self.marker.color('red')
        self.sim = True
        self.timer = 0

    def cycle(self):
        while(self.sim):
            thrust = self.pid.compute(self.Insight.get_y())
            self.Insight.set_ddy(thrust)
            self.Insight.set_dy()
            self.Insight.set_y()
            time.sleep(TIME_STEP)
            self.timer += 1

            if self.timer > SIM_TIME:
                print("SIM ENDED")
                self.sim = False
            elif self.Insight.get_y() > 800:
                print("POSITIVE OUT OF BOUND")
                self.sim = False
            elif self.Insight.get_y() < -800:
                print("NEGATIVE OUT OF BOUND")
                self.sim = False

class Rocket():
    def __init__(self) -> None:
        global Rocket
        self.Rocket = turtle.Turtle()
        self.Rocket.shape('square')
        self.Rocket.color('black')
        self.Rocket.penup()
        self.Rocket.goto(INITIAL_X, INITIAL_Y)
        self.Rocket.speed(0)

        self.ddy = 0        # acceleration
        self.dy = V_i      # velocity
        self.y = INITIAL_Y

    def set_ddy(self, thrust):          # thrust <- PID Controller
        self.ddy = g + (thrust / MASS)

    def get_ddy(self):
        return self.ddy

    def set_dy(self):
        self.dy += self.ddy

    def get_dy(self):
        return self.dy

    def set_y(self):
        self.Rocket.sety(self.y + self.dy)

    def get_y(self):
        self.y = self.Rocket.ycor()
        return self.y


class PID():
    def __init__(self, KP, KI, KD, target):
        self.kp = KP
        self.ki = KI
        self.kd = KD

        self.setpoint = target
        self.error = 0
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0
        self.output = 0

    def compute(self, pos):
        self.error = self.setpoint - pos
        self.integral_error += self.error * TIME_STEP
        self.derivative_error = (self.error - self.error_last)/TIME_STEP
        self.error_last = self.error

        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
        if self.output >= MAX_TRUST:
            self.output = MAX_TRUST
        if self.output <= 0:
            self.output = 0
        return self.output


def main():
    sim = Simulation()
    sim.cycle()

main()